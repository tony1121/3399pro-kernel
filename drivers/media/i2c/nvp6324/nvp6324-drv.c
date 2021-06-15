// SPDX-License-Identifier: GPL-2.0
/*
 * nvp6324 driver
 *
 * Copyright (C) 2019 Kaspter Ju camus@rtavs.com.
 */


#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>


#define REG_NULL            0x5a
#define REG_DELAY           0xa5


#define NVP6324_XVCLK_FREQ		27000000
#define NVP6324_LINK_FREQ		320000000

/* pixel_rate = link_freq * 2 * nr_of_lanes / bits_per_sample */
#define NVP6324_PIXEL_RATE		(NVP6324_LINK_FREQ * 2 * 4 / 8)
static const s64 link_freq_menu_items[] = {
	NVP6324_LINK_FREQ
};


static const char * const nvp6324_supply_names[] = {
    "dvdd18",
    "dvdd12",
    "avdd33",
};

#define NVP6324_NUM_SUPPLIES ARRAY_SIZE(nvp6324_supply_names)



struct regval {
    u8 addr;
    u8 val;
};


struct nvp6324_mode {
    u32 width;
    u32 height;
    u32 interlace;
    const struct regval *reg_list;
};

struct nvp6324 {
    struct i2c_client   *client;
    struct clk          *xvclk;
    struct gpio_desc    *reset_gpio;

    struct regulator_bulk_data supplies[NVP6324_NUM_SUPPLIES];

    bool                streaming;
    struct mutex        mutex; /* lock to serialize v4l2 callback */
    struct v4l2_subdev  subdev;
    struct media_pad    pad;

    struct v4l2_ctrl_handler ctrl_handler;

    int         skip_top;

    v4l2_std_id             std; /* Current set standard */
    const struct nvp6324_mode *cur_mode;
};

#define to_nvp6324(sd) container_of(sd, struct nvp6324, subdev)



static struct regval nvp6324_mipi_stop[] = {
    // arb disable
    {0xff, 0x20},
    {0x00, 0x00},
    {0xff, 0x20},

    // arb reset high
    {0x40, 0x11},
    {REG_DELAY, 0x04},
    {0x40, 0x10},

    {0xff, 0x00},
    {0x4e, 0x7f},
    {REG_NULL, 0x0},
};



static int nvp6324_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
    int ret;

    ret = i2c_smbus_write_byte_data(client, reg, val);

    if (ret < 0)
        dev_err(&client->dev, "write reg error: %d\n", ret);

    return ret;
}

static int nvp6324_write_array(struct i2c_client *client,
                   const struct regval *regs)
{
    int i, ret = 0;

    for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
        ret = nvp6324_write_reg(client, regs[i].addr, regs[i].val);

    return ret;
}

static inline u8 nvp6324_read_reg(struct i2c_client *client, u8 reg)
{
    return i2c_smbus_read_byte_data(client, reg);
}


static int nvp6324_configure_regulators(struct nvp6324 *nvp6324)
{
    u32 i;

    for (i = 0; i < NVP6324_NUM_SUPPLIES; i++)
        nvp6324->supplies[i].supply = nvp6324_supply_names[i];

    return devm_regulator_bulk_get(&nvp6324->client->dev,
                       NVP6324_NUM_SUPPLIES,
                       nvp6324->supplies);
}


static int __nvp6324_power_on(struct nvp6324 *nvp6324)
{
    int ret;
    struct device *dev = &nvp6324->client->dev;

    if (!IS_ERR(nvp6324->xvclk)) {
        ret = clk_prepare_enable(nvp6324->xvclk);
        if (ret < 0) {
            dev_err(dev, "Failed to enable xvclk\n");
            return ret;
        }
    }

    if (!IS_ERR(nvp6324->reset_gpio))
        gpiod_set_value_cansleep(nvp6324->reset_gpio, 1);

    ret = regulator_bulk_enable(NVP6324_NUM_SUPPLIES, nvp6324->supplies);
    if (ret < 0) {
        dev_err(dev, "Failed to enable regulators\n");
        goto disable_clk;
    }

    msleep(2);
    if (!IS_ERR(nvp6324->reset_gpio))
        gpiod_set_value_cansleep(nvp6324->reset_gpio, 0);

    return 0;

disable_clk:
    if (!IS_ERR(nvp6324->xvclk))
        clk_disable_unprepare(nvp6324->xvclk);

    return ret;
}

static void __nvp6324_power_off(struct nvp6324 *nvp6324)
{
    if (!IS_ERR(nvp6324->xvclk))
        clk_disable_unprepare(nvp6324->xvclk);
    if (!IS_ERR(nvp6324->reset_gpio))
        gpiod_set_value_cansleep(nvp6324->reset_gpio, 1);
    regulator_bulk_disable(PR2000_NUM_SUPPLIES, nvp6324->supplies);
}

//FIXME
static void nvp6324_soft_reset(struct nvp6324 *nvp6324)
{
    struct i2c_client *client = nvp6324->client;

    nvp6324_write_reg(client, 0xff, 0x00);
    nvp6324_write_reg(client, 0xf8, 0xff);
    usleep_range(80000, 100000);
    nvp6324_write_reg(client, 0xf8, 0x00);
}




static int nvp6324_init_ctrls(struct nvp6324 *nvp6324)
{
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	int ret;

	handler = &nvp6324->ctrl_handler;
	ret = v4l2_ctrl_handler_init(handler, 2);
	if (ret)
		return ret;
	handler->lock = &nvp6324->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, NVP6324_PIXEL_RATE, 1, NVP6324_PIXEL_RATE);

	if (handler->error) {
		ret = handler->error;
		dev_err(&nvp6324->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	nvp6324->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}



static int nvp6324_s_stream(struct v4l2_subdev *sd, int on)
{
    struct nvp6324 *nvp6324 = to_nvp6324(sd);
    struct i2c_client *client = nvp6324->client;
    int ret = 0;

    mutex_lock(&nvp6324->mutex);

    on = !!on;
    if (on == nvp6324->streaming)
        goto unlock_and_return;

    if (on) {
        ret = pm_runtime_get_sync(&nvp6324->client->dev);
        if (ret < 0) {
            pm_runtime_put_noidle(&client->dev);
            goto unlock_and_return;
        }

        nvp6324_soft_reset(nvp6324);

        ret = nvp6324_write_array(nvp6324->client, nvp6324->cur_mode->reg_list);
        if (ret) {
            pm_runtime_put(&client->dev);
            goto unlock_and_return;
        }

        ret = nvp6324_write_array(nvp6324->client, nvp6324_mipi_start);
        if (ret) {
            pm_runtime_put(&client->dev);
            goto unlock_and_return;
        }

        if (nvp6324->cur_mode->interlace) {
            ret = nvp6324_write_reg(client, 0x47, 0xb6);
        } else {
            ret = nvp6324_write_reg(client, 0x47, 0xba);
        }
        if (ret) {
            pm_runtime_put(&client->dev);
            goto unlock_and_return;
        }
        nvp6324_dump_status(nvp6324);
    } else {
        ret = nvp6324_write_array(nvp6324->client, nvp6324_mipi_stop);
        pm_runtime_put(&client->dev);
    }

    nvp6324->streaming = on;

unlock_and_return:
    mutex_unlock(&nvp6324->mutex);

    return ret;
}



static int nvp6324_probe(struct i2c_client *client,
             const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct nvp6324 *nvp6324;
    int ret;

    nvp6324 = devm_kzalloc(dev, sizeof(*nvp6324), GFP_KERNEL);
    if (!nvp6324)
        return -ENOMEM;

    nvp6324->client = client;
    nvp6324->cur_mode = &supported_modes[0];

    nvp6324->xvclk = devm_clk_get(dev, "xvclk");
    if (!IS_ERR(nvp6324->xvclk)) {
        ret = clk_set_rate(nvp6324->xvclk, NVP6324_XVCLK_FREQ);
        if (ret < 0) {
            dev_err(dev, "Failed to set xvclk rate (27MHz)\n");
            return ret;
        }
        if (clk_get_rate(nvp6324->xvclk) != NVP6324_XVCLK_FREQ)
            dev_warn(dev, "xvclk mismatched, it requires 27MHz\n");
    }

    nvp6324->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(nvp6324->reset_gpio)) {
		dev_warn(dev, "Failed to get reset-gpios\n");
    }

    ret = nvp6324_configure_regulators(nvp6324);
    if (ret) {
        dev_err(dev, "Failed to get power regulators\n");
        return ret;
    }

    mutex_init(&nvp6324->mutex);
    v4l2_i2c_subdev_init(&nvp6324->subdev, client, &nvp6324_subdev_ops);
    ret = nvp6324_init_ctrls(nvp6324);
    if (ret)
        goto err_destroy_mutex;

    ret = __nvp6324_power_on(nvp6324);
    if (ret)
        goto err_destroy_mutex;

    ret = nvp6324_check_chipid(nvp6324);
    if (ret)
        goto err_power_off;

    ret = nvp6324_check_mode(nvp6324);
    if (ret)
        goto err_power_off;

    nvp6324->subdev.internal_ops = &nvp6324_internal_ops;
    nvp6324->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    nvp6324->pad.flags = MEDIA_PAD_FL_SOURCE;
    nvp6324->subdev.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
    ret = media_entity_init(&nvp6324->subdev.entity, 1, &nvp6324->pad, 0);
    if (ret < 0)
        goto err_power_off;

    ret = v4l2_async_register_subdev(&nvp6324->subdev);
    if (ret) {
        dev_err(dev, "v4l2 async register subdev failed\n");
        goto err_clean_entity;
    }

    pm_runtime_set_active(dev);
    pm_runtime_enable(dev);
    pm_runtime_idle(dev);

    return 0;

err_clean_entity:
    media_entity_cleanup(&nvp6324->subdev.entity);
err_power_off:
    __nvp6324_power_off(nvp6324);
err_destroy_mutex:
    mutex_destroy(&nvp6324->mutex);

    return ret;
}

static int nvp6324_remove(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct nvp6324 *nvp6324 = to_nvp6324(sd);

    v4l2_async_unregister_subdev(sd);
    media_entity_cleanup(&sd->entity);
    mutex_destroy(&nvp6324->mutex);

    pm_runtime_disable(&client->dev);
    if (!pm_runtime_status_suspended(&client->dev))
        __nvp6324_power_off(nvp6324);
    pm_runtime_set_suspended(&client->dev);

    return 0;
}




static const struct i2c_device_id nvp6324_id[] = {
    {"nvp6324", 0},
    {},
};

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id nvp6324_of_match[] = {
    { .compatible = "nextchip,nvp6324" },
    {},
};
MODULE_DEVICE_TABLE(of, nvp6324_of_match);
#endif

static struct i2c_driver nvp6324_i2c_driver = {
    .driver = {
        .name = "nvp6324",
        .pm = &nvp6324_pm_ops,
        .of_match_table = nvp6324_of_match
    },
    .probe      = nvp6324_probe,
    .remove     = nvp6324_remove,
    .id_table   = nvp6324_id,
};

static int __init nvp6324_init(void)
{
	return i2c_add_driver(&nvp6324_i2c_driver);
}

static void __exit nvp6324_exit(void)
{
	i2c_del_driver(&nvp6324_i2c_driver);
}

device_initcall_sync(nvp6324_init);

MODULE_DESCRIPTION("nvp6324 sensor driver");
MODULE_LICENSE("GPL v2");
