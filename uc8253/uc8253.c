#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/stdarg.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>

#include <linux/wait.h>
#include <linux/spinlock.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/regmap.h>

#include <linux/uaccess.h>

#include <linux/fb.h>
#include <video/mipi_display.h>

#include "epd_3in52_regs.h"
#include "bootlogo.h"

#define DRV_NAME "uc8253_drv"

struct uc8253_data;

struct uc8253_operations {
        int (*refresh)(struct uc8253_data *ud, u8 mode);
        int (*reset)(struct uc8253_data *ud);
        int (*clear)(struct uc8253_data *ud);
        int (*sleep)(struct uc8253_data *ud);
        int (*is_busy)(struct uc8253_data *ud);
};

struct uc8253_data {

        struct device           *dev;
        struct spi_device       *spi;
        // u8                      *buf;
        // struct {
        //         void *buf;
        //         size_t len;
        // } txbuf;
        struct {
                struct gpio_desc *reset;
                struct gpio_desc *dc;
                struct gpio_desc *cs;
                struct gpio_desc *busy;
        } gpio;
        struct completion       complete;

        /* device specific */
        u32                     xres;
        u32                     yres;
        u32                     refr_mode;
        u32                     wait;
        u32                     busy;

        struct uc8253_operations        ops;

        struct fb_info          *fbinfo;
        struct fb_ops           *fbops;
};

static int g_epd_3in52_flag = 0;

static int fbtft_write_spi(struct uc8253_data *ud, void *buf, size_t len)
{
        struct spi_transfer t = {
                .tx_buf = buf,
                .len = len,
        };
        struct spi_message m;

        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
        return spi_sync(ud->spi, &m);
}

static int fbtft_write_buf_dc(struct uc8253_data *ud, void *buf, size_t len, int dc)
{
        int rc;

        gpiod_set_value(ud->gpio.dc, dc);

        rc = fbtft_write_spi(ud, buf, len);
        if (rc < 0)
                dev_err(ud->dev, "write() failed and returned %d\n", rc);

        return rc;
}

static __inline int uc8253_send(struct uc8253_data *ud, u8 byte, int dc)
{
        fbtft_write_buf_dc(ud, &byte, 1, dc);
        return 0;
}
#define UC8253_DC_DATA          1
#define UC8253_DC_COMMAND       0
#define write_cmd(__ud, __c) uc8253_send(__ud, __c, UC8253_DC_COMMAND);
#define write_data(__ud, __d) uc8253_send(__ud, __d, UC8253_DC_DATA);

static int uc8253_init_display(struct uc8253_data *ud)
{
        write_cmd(ud, 0x00);
        write_data(ud, 0xFF);
        write_data(ud, 0x01);

        write_cmd(ud, 0x01);
        write_data(ud, 0x03);
        write_data(ud, 0x10);
        write_data(ud, 0x3F);
        write_data(ud, 0x3F);
        write_data(ud, 0x03);

        write_cmd(ud, 0x06);
        write_data(ud, 0x37);
        write_data(ud, 0x3D);
        write_data(ud, 0x3D);

        write_cmd(ud, 0x60);
        write_data(ud, 0x22);

        write_cmd(ud, 0x82);
        write_data(ud, 0x07);

        write_cmd(ud, 0x30);
        write_data(ud, 0x09)   // 50Hz
        // write_data(ud, 0x13)   // 100Hz
        // write_data(ud, 0x17)   // 120Hz

        write_cmd(ud, 0xe3);
        write_data(ud, 0x88);

        write_cmd(ud, 0x61);
        write_data(ud, 0xf0);
        write_data(ud, 0x01);
        write_data(ud, 0x68);

        write_cmd(ud, 0x50);
        write_data(ud, 0xB7);

        return 0;
}

#define uc8253_load_loop(__ud, reg, arr) \
        write_cmd(__ud, reg) \
        for (i = 0; i < ARRAY_SIZE(arr); i++)   \
                write_data(__ud, arr[i])

static int uc8253_load_lut_gc(struct uc8253_data *ud)
{
        int i;

        uc8253_load_loop(ud, 0x20, epd_3in52_lut_r20_gc);
        uc8253_load_loop(ud, 0x21, epd_3in52_lut_r21_gc);
        uc8253_load_loop(ud, 0x24, epd_3in52_lut_r24_gc);

        if (g_epd_3in52_flag == 0) {
                uc8253_load_loop(ud, 0x22, epd_3in52_lut_r22_gc);
                uc8253_load_loop(ud, 0x23, epd_3in52_lut_r23_gc);
        } else {
                uc8253_load_loop(ud, 0x22, epd_3in52_lut_r23_gc);
                uc8253_load_loop(ud, 0x23, epd_3in52_lut_r22_gc);
        }
        g_epd_3in52_flag = !g_epd_3in52_flag;
        return 0;
}

static int uc8253_reset(struct uc8253_data *ud)
{
        gpiod_set_value_cansleep(ud->gpio.reset, 1);
        msleep(10);
        gpiod_set_value_cansleep(ud->gpio.reset, 0);
        msleep(200);
        gpiod_set_value_cansleep(ud->gpio.reset, 1);
        msleep(10);

        return 0;
}

static int uc8253_refresh(struct uc8253_data *ud, u8 mode)
{
        write_cmd(ud, 0x17);
        write_data(ud, 0xa5);
        mdelay(200);
        return 0;
}

static int uc8253_clear(struct uc8253_data *ud)
{
        int i;
        write_cmd(ud, 0x13);
        for (i = 0; i < 360 * 240 / 8; i++)
                write_data(ud, 0xff);

        uc8253_load_lut_gc(ud);
        uc8253_refresh(ud, 0);

        return 0;
}

static const struct uc8253_operations default_uc8253_ops = {
        .refresh = uc8253_refresh,
        .clear = uc8253_clear,
        .reset = NULL,
        .is_busy = NULL,
        .sleep = NULL,
};

static const int uc8253_show_img(struct uc8253_data *ud, const u8 *img)
{
        int i;
        write_cmd(ud, 0x13);
        for (i = 0; i < 360 * 240 / 8; i++)
                write_data(ud, img[i]);

        uc8253_load_lut_gc(ud);
        uc8253_refresh(ud, 0);
        return 0;
}

static int uc8253_request_one_gpio(struct uc8253_data *ud,
                                   const char *name, int index,
                                   struct gpio_desc **gpiop)
{
        struct device *dev = ud->dev;
        struct device_node *np = dev->of_node;
        int gpio, flags, rc = 0;
        enum of_gpio_flags of_flags;

        if (of_find_property(np, name, NULL)) {
                gpio = of_get_named_gpio_flags(np, name, index, &of_flags);
                if (gpio == -ENOENT)
                        return 0;
                if (gpio == -EPROBE_DEFER)
                        return gpio;
                if (gpio < 0) {
                        dev_err(dev,
                                "failed to get '%s' from DT\n", name);
                        return gpio;
                }

                flags = (of_flags & OF_GPIO_ACTIVE_LOW) ? GPIOF_OUT_INIT_LOW :
                        GPIOF_OUT_INIT_HIGH;
                rc = devm_gpio_request_one(dev, gpio, flags,
                                           dev->driver->name);
                if (rc) {
                        dev_err(dev,
                                "gpio_request_one('%s'=%d) failed with %d\n",
                                name, gpio, rc);
                        return rc;
                }
                if (gpiop)
                        *gpiop = gpio_to_desc(gpio);
                pr_debug("%s : '%s' = GPIO%d\n",
                         __func__, name, gpio);
        }

        return rc;
}

static int uc8253_request_gpios(struct uc8253_data *ud)
{
        int rc;
        pr_debug("%s, configure from dt\n", __func__);

        rc = uc8253_request_one_gpio(ud, "reset-gpios", 0, &ud->gpio.reset);
        if (rc)
                return rc;
        rc = uc8253_request_one_gpio(ud, "dc-gpios", 0, &ud->gpio.dc);
        if (rc)
                return rc;
        rc = uc8253_request_one_gpio(ud, "busy-gpios", 0, &ud->gpio.busy);
        if (rc)
                return rc;
        rc = uc8253_request_one_gpio(ud, "cs-gpios", 0, &ud->gpio.cs);
        if (rc)
                return rc;

        return 0;
}

static int uc8253_of_config(struct uc8253_data *ud)
{
        int rc;

        printk("%s\n", __func__);
        rc = uc8253_request_gpios(ud);
        if (rc) {
                dev_err(ud->dev, "Request gpios failed!\n");
                return rc;
        }
        return 0;

        /* request xres and yres from dt */

}

static int uc8253_hw_init(struct uc8253_data *ud)
{
        printk("%s, Display Panel initializing ...\n", __func__);
        uc8253_reset(ud);
        uc8253_init_display(ud);
        // uc8253_set_addr_win(ud, 0, 0, 200, 200);
        uc8253_clear(ud);

        uc8253_show_img(ud, internal_bootup_logo);
        uc8253_load_lut_gc(ud);
        uc8253_refresh(ud, 0);

        uc8253_show_img(ud, internal_bootup_logo);
        uc8253_load_lut_gc(ud);
        uc8253_refresh(ud, 0);

        return 0;
}


static int uc8253_probe(struct spi_device *spi)
{
        struct uc8253_data *ud;
        struct device *dev = &spi->dev;
        struct fb_info *info;

        printk("%s\n", __func__);
        /* memory resource alloc */
        // ud = kmalloc(sizeof(struct uc8253_data), GFP_KERNEL);
        // if (!ud) {
        //         dev_err(dev, "failed to alloc ud memory!\n");
        //         return -ENOMEM;
        // }

        // ud->buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
        // if (!ud->buf) {
        //         dev_err(dev, "failed to alloc buf memory!\n");
        //         return -ENOMEM;
        // }

        // ud->txbuf.buf = kmalloc(SPI_BUF_LEN, GFP_KERNEL);
        // if (!ud->txbuf.buf) {
        //         dev_err(dev, "failed to alloc txbuf!\n");
        //         return -ENOMEM;
        // }
        
        info = framebuffer_alloc(sizeof(struct uc8253_data), dev);
        if (!info) {
                dev_err(dev, "failed to alloc framebuffer!\n");
                return -ENOMEM;
        }

        ud = info->par;

        ud->fbinfo = info;
        ud->spi = spi;
        ud->dev = dev;
        spi_set_drvdata(spi, ud);

        init_completion(&ud->complete);

        uc8253_of_config(ud);
        uc8253_hw_init(ud);

        return 0;
}

static int uc8253_remove(struct spi_device *spi)
{
        struct uc8253_data *ud = spi_get_drvdata(spi);

        printk("%s\n", __func__);
        // kfree(ud->buf);
        // kfree(ud->txbuf.buf);
        framebuffer_release(ud->fbinfo);

        kfree(ud);
        return 0;
}

static const struct of_device_id uc8253_dt_ids[] = {
        { .compatible = "ultrachip,uc8253" },
        { /* KEEP THIS */ },
};
MODULE_DEVICE_TABLE(of, uc8253_dt_ids);

static const struct spi_device_id uc8253_spi_ids[] = {
        { "uc8253" },
        { /* KEEP THIS */ },
};
MODULE_DEVICE_TABLE(spi, uc8253_spi_ids);

static struct spi_driver uc8253_drv = {
        .driver = {
                .name = DRV_NAME,
                .of_match_table = of_match_ptr(uc8253_dt_ids),
        },
        .id_table = uc8253_spi_ids,
        .probe = uc8253_probe,
        .remove = uc8253_remove,
};

module_spi_driver(uc8253_drv);

MODULE_AUTHOR("Iota Hydrae <writeforever@foxmail.com>");
MODULE_DESCRIPTION("UC8253 E-Paper display driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:uc8253");
