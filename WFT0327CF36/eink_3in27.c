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

#include "eink_3in27_regs.h"
#include "imagedata.h"

#define DRV_NAME "eink_3in27_drv"

struct eink_3in27_par;

struct eink_3in27_operations {
        int (*refresh)(struct eink_3in27_par *par, u8 mode);
        int (*reset)(struct eink_3in27_par *par);
        int (*clear)(struct eink_3in27_par *par);
        int (*sleep)(struct eink_3in27_par *par);
        int (*is_busy)(struct eink_3in27_par *par);
};

struct eink_3in27_display {
        u32                     xres;
        u32                     yres;
        u32                     bpp;
};

struct eink_3in27_par {

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

        spinlock_t              dirty_lock;
        struct completion       complete;

        /* device specific */
        u32                     refr_mode;
        u32                     wait;
        u32                     busy;

        struct eink_3in27_operations        ops;
        const struct eink_3in27_display           *display;

        struct tasklet_struct task;

        struct fb_info          *fbinfo;
        struct fb_ops           *fbops;

        u32             palette_buffer[256];
        u32             palette_pal[16];
};

// static int g_epd_3in27_flag = 0;
u32 pseudo_palette[16];

static int fbtft_write_spi(struct eink_3in27_par *par, void *buf, size_t len)
{
        struct spi_transfer t = {
                .tx_buf = buf,
                .len = len,
        };
        struct spi_message m;

        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
        return spi_sync(par->spi, &m);
}

static int fbtft_write_buf_dc(struct eink_3in27_par *par, void *buf, size_t len, int dc)
{
        int rc;

        gpiod_set_value(par->gpio.dc, dc);

        rc = fbtft_write_spi(par, buf, len);
        if (rc < 0)
                dev_err(par->dev, "write() failed and returned %d\n", rc);

        return rc;
}

static __inline int eink_3in27_send(struct eink_3in27_par *par, u8 byte, int dc)
{
        fbtft_write_buf_dc(par, &byte, 1, dc);
        return 0;
}
#define eink_3in27_DC_DATA          1
#define eink_3in27_DC_COMMAND       0
#define write_cmd(__par, __c) eink_3in27_send(__par, __c, eink_3in27_DC_COMMAND);
#define write_data(__par, __d) eink_3in27_send(__par, __d, eink_3in27_DC_DATA);

static int eink_3in27_reset(struct eink_3in27_par *par)
{
        gpiod_set_value_cansleep(par->gpio.reset, 1);
        msleep(10);
        gpiod_set_value_cansleep(par->gpio.reset, 0);
        msleep(100);
        gpiod_set_value_cansleep(par->gpio.reset, 1);
        msleep(10);
        return 0;
}

static int eink_3in27_read_busy(struct eink_3in27_par *par)
{
        write_cmd(par, 0x71);
        return 0;
}

#define eink_3in27_load_loop(__par, reg, arr) \
        write_cmd(__par, reg) \
        for (i = 0; i < ARRAY_SIZE(arr); i++)   \
                write_data(__par, arr[i])

static int eink_3in27_load_reg(struct eink_3in27_par *par)
{
        int i;

        eink_3in27_load_loop(par, 0x20, eink_3in27_lut_vcom_dc);
        eink_3in27_load_loop(par, 0x21, eink_3in27_lut_ww);
        eink_3in27_load_loop(par, 0x22, eink_3in27_lut_bw);
        eink_3in27_load_loop(par, 0x23, eink_3in27_lut_wb);
        eink_3in27_load_loop(par, 0x24, eink_3in27_lut_bb);

        return 0;
}

static int eink_3in27_init_display(struct eink_3in27_par *par)
{
        eink_3in27_reset(par);

        write_cmd(par, 0x01);
        write_data(par, 0x07);
        write_data(par, 0x00);
        write_data(par, 0x0A);
        write_data(par, 0x00);

        write_cmd(par, 0x06);
        write_data(par, 0x07);
        write_data(par, 0x07);
        write_data(par, 0x07);

        write_cmd(par, 0x04);
        eink_3in27_read_busy(par);
        mdelay(100);

        write_cmd(par, 0x00);
        write_data(par, 0xCf);
        write_cmd(par, 0x50);
        write_data(par, 0x37);
        write_cmd(par, 0x30);
        write_data(par, 0x39);
        write_cmd(par, 0x61);
        write_data(par, 0xC8);
        write_data(par, 0x01);
        write_data(par, 0x2C);
        write_cmd(par, 0X82);
        write_data(par, 0x0C);

        eink_3in27_load_reg(par);
        return 0;
}

static int eink_3in27_turn_on_display(struct eink_3in27_par *par, u8 mode)
{
        write_cmd(par, 0x12);
        mdelay(100);
        /* TODO: wait busy here */
        eink_3in27_read_busy(par);
        return 0;
}

static int eink_3in27_sleep(struct eink_3in27_par *par)
{
        write_cmd(par, 0x50);
        write_data(par, 0x37);
        write_cmd(par, 0x02);
        write_cmd(par, 0x07);
        write_data(par, 0xa5);

        return 0;
}

static int eink_3in27_clear(struct eink_3in27_par *par)
{
        int i;
        write_cmd(par, 0x10);
        for (i = 0; i < 15000; i++) {
                write_data(par, 0x00);
        }
        write_cmd(par, 0x10);
        for (i = 0; i < 15000; i++) {
                write_data(par, 0xff);
        }

        return 0;
}

static const struct eink_3in27_operations default_eink_3in27_ops = {
        .refresh = eink_3in27_turn_on_display,
        .clear = eink_3in27_clear,
        .reset = NULL,
        .is_busy = NULL,
        .sleep = NULL,
};

static const int eink_3in27_show_img(struct eink_3in27_par *par, const u8 *img)
{
        int i;

        return 0;
}

static int eink_3in27_request_one_gpio(struct eink_3in27_par *par,
                                       const char *name, int index,
                                       struct gpio_desc **gpiop)
{
        struct device *dev = par->dev;
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

static int eink_3in27_request_gpios(struct eink_3in27_par *par)
{
        int rc;
        pr_debug("%s, configure from dt\n", __func__);

        rc = eink_3in27_request_one_gpio(par, "reset-gpios", 0, &par->gpio.reset);
        if (rc)
                return rc;
        rc = eink_3in27_request_one_gpio(par, "dc-gpios", 0, &par->gpio.dc);
        if (rc)
                return rc;
        rc = eink_3in27_request_one_gpio(par, "busy-gpios", 0, &par->gpio.busy);
        if (rc)
                return rc;
        rc = eink_3in27_request_one_gpio(par, "cs-gpios", 0, &par->gpio.cs);
        if (rc)
                return rc;

        return 0;
}

/* returns 0 if the property is not present */
static u32 fbtft_property_value(struct device *dev, const char *propname)
{
        int ret;
        u32 val = 0;

        ret = device_property_read_u32(dev, propname, &val);
        if (ret == 0)
                dev_info(dev, "%s: %s = %u\n", __func__, propname, val);

        return val;
}

static int eink_3in27_of_config(struct eink_3in27_par *par)
{
        int rc;

        printk("%s\n", __func__);
        rc = eink_3in27_request_gpios(par);
        if (rc) {
                dev_err(par->dev, "Request gpios failed!\n");
                return rc;
        }
        return 0;

        /* request xres and yres from dt */
}

static int eink_3in27_hw_init(struct eink_3in27_par *par)
{
        printk("%s, Display Panel initializing ...\n", __func__);
        eink_3in27_init_display(par);
        eink_3in27_clear(par);
        eink_3in27_turn_on_display(par, 0);

        // tasklet_enable(&par->task);
        return 0;
}

static const struct eink_3in27_display display = {
        .xres = 300,
        .yres = 200,
        .bpp = 1,
};

static irqreturn_t panel_busy_handle(int irq, void *data)
{
        return IRQ_HANDLED;
}

static int eink_3in27_setup_irq(struct eink_3in27_par *par)
{
        struct device *dev = par->dev;
        struct gpio_desc *busy;

        busy = devm_gpiod_get_optional(dev, "busy", GPIOD_IN);
        if (IS_ERR(busy))
                return dev_err_probe(dev, PTR_ERR(busy), "Failed to request the busy GPIO\n");

        if (!busy)
                return 0;

        return 0;
}

static void update_display(struct eink_3in27_par *par)
{
        int i;
        u8 *buf = par->fbinfo->screen_buffer;

        // printk("%s\n", __func__);
        dev_dbg(par->dev, "%s\n", __func__);
        /* write vmem to display then call refresh routine */
        /*
         * when this was called, driver should wait for busy pin comes low
         * until next frame refreshed
         */
}

static void eink_3in27_flush_task(unsigned long data)
{
        // int timeout;
        struct eink_3in27_par *par = (struct eink_3in27_par *)data;
}

static void eink_3in27_mkdirty(struct fb_info *info, int y, int height)
{
        struct eink_3in27_par *par = info->par;
        // struct fb_deferred_io *fbdefio = info->fbdefio;
        dev_dbg(info->dev, "%s\n", __func__);
        // spin_lock(&par->dirty_lock);
        // spin_unlock(&par->dirty_lock);

        // schedule_delayed_work(&info->deferred_work, 0);
        // tasklet_schedule(&par->task);
}

static void eink_3in27_deferred_io(struct fb_info *info, struct list_head *pagelist)
{
        struct eink_3in27_par *par = info->par;
}

static ssize_t eink_3in27_fb_write(struct fb_info *info, const char __user *buf,
                                   size_t count, loff_t *ppos)
{
        ssize_t res;
        dev_dbg(info->dev,
                "%s: count=%zd, ppos=%llu\n", __func__,  count, *ppos);

        res = fb_sys_write(info, buf, count, ppos);
        eink_3in27_mkdirty(info, -1, 0);
        return 0;
}

static void eink_3in27_fb_fillrect(struct fb_info *info,
                                   const struct fb_fillrect *rect)
{
        dev_dbg(info->dev,
                "%s: dx=%d, dy=%d, width=%d, height=%d\n",
                __func__, rect->dx, rect->dy, rect->width, rect->height);

        sys_fillrect(info, rect);
        eink_3in27_mkdirty(info, rect->dy, rect->height);
}

static void eink_3in27_fb_copyarea(struct fb_info *info,
                                   const struct fb_copyarea *area)
{
        dev_dbg(info->dev,
                "%s: dx=%d, dy=%d, width=%d, height=%d\n",
                __func__,  area->dx, area->dy, area->width, area->height);

        sys_copyarea(info, area);
        eink_3in27_mkdirty(info, area->dy, area->height);
}

static void eink_3in27_fb_imageblit(struct fb_info *info,
                                    const struct fb_image *image)
{
        dev_dbg(info->dev,
                "%s: dx=%d, dy=%d, width=%d, height=%d\n",
                __func__,  image->dx, image->dy, image->width, image->height);
        sys_imageblit(info, image);

        eink_3in27_mkdirty(info, image->dy, image->height);
}

static int eink_3in27_fb_setcolreg(unsigned int regno, unsigned int red,
                                   unsigned int green, unsigned int blue,
                                   unsigned int transp, struct fb_info *info)
{
        int ret = 1;
        // u8 val;

        dev_dbg(info->dev,
                "%s(regno=%u, red=0x%X, green=0x%X, blue=0x%X, trans=0x%X)\n",
                __func__, regno, red, green, blue, transp);

        switch (info->fix.visual) {
        case FB_VISUAL_MONO01:
                dev_dbg(info->dev, "FB_VISUAL_MONO01\n");
                break;
        }

        return ret;
}

static int eink_3in27_probe(struct spi_device *spi)
{
        struct device *dev = &spi->dev;
        struct eink_3in27_par *par;
        struct fb_deferred_io *fbdefio;
        struct fb_info *info;
        struct fb_ops *fbops;
        u8 *vmem = NULL;
        int vmem_size;
        int rc;

        printk("%s\n", __func__);
        /* memory resource alloc */
        // display = kmalloc(sizeof(struct eink_3in27_display), GFP_KERNEL);
        // if (!display) {
        //         dev_err(dev, "failed to alloc par memory!\n");
        //         return -ENOMEM;
        // }

        // par->buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
        // if (!par->buf) {
        //         dev_err(dev, "failed to alloc buf memory!\n");
        //         return -ENOMEM;
        // }

        // par->txbuf.buf = kmalloc(SPI_BUF_LEN, GFP_KERNEL);
        // if (!par->txbuf.buf) {
        //         dev_err(dev, "failed to alloc txbuf!\n");
        //         return -ENOMEM;
        // }

        vmem_size = display.xres * display.yres * display.bpp / BITS_PER_BYTE;
        dev_dbg(dev, "vmem_size : %d\n", vmem_size);
        vmem = vzalloc(vmem_size);
        if (!vmem)
                goto alloc_fail;

        fbops = devm_kzalloc(dev, sizeof(struct fb_ops), GFP_KERNEL);
        if (!fbops)
                goto alloc_fail;

        fbdefio = devm_kzalloc(dev, sizeof(struct fb_deferred_io), GFP_KERNEL);
        if (!fbdefio)
                goto alloc_fail;

        /* framebuffer info setup */
        info = framebuffer_alloc(sizeof(struct eink_3in27_par), dev);
        if (!info) {
                dev_err(dev, "failed to alloc framebuffer!\n");
                return -ENOMEM;
        }

        info->screen_buffer = vmem;
        info->fbops = fbops;
        info->fbdefio = fbdefio;

        fbops->owner = dev->driver->owner;
        fbops->fb_read = fb_sys_read;
        fbops->fb_write = eink_3in27_fb_write;
        fbops->fb_fillrect = eink_3in27_fb_fillrect;
        fbops->fb_copyarea = eink_3in27_fb_copyarea;
        fbops->fb_imageblit = eink_3in27_fb_imageblit;
        // fbops->fb_cursor = NULL;
        fbops->fb_setcolreg = eink_3in27_fb_setcolreg;
        // fbops->fb_blank = NULL;

        // fbdefio->delay = HZ;
        // fbdefio->deferred_io = eink_3in27_deferred_io;
        // fb_deferred_io_init(info);

        snprintf(info->fix.id, sizeof(info->fix.id), "%s", dev->driver->name);
        info->fix.type            =       FB_TYPE_PACKED_PIXELS;
        info->fix.visual          =       FB_VISUAL_MONO01;
        info->fix.xpanstep        =       0;
        info->fix.ypanstep        =       1;
        info->fix.ywrapstep       =       0;
        info->fix.accel           =       FB_ACCEL_NONE;
        info->fix.line_length     =       display.xres * display.bpp / BITS_PER_BYTE;
        info->fix.smem_len        =       vmem_size;

        info->var.rotate          =       0;
        info->var.xres            =       display.xres;
        info->var.yres            =       display.yres;
        info->var.xres_virtual    =       info->var.xres;
        info->var.yres_virtual    =       info->var.yres;
        info->var.bits_per_pixel  =       display.bpp;
        info->var.nonstd          =       1;

        info->var.grayscale     = 1;
        info->var.transp.offset = 0;
        info->var.transp.length = 0;

        info->flags = FBINFO_FLAG_DEFAULT | FBINFO_VIRTFB;
        info->pseudo_palette = &pseudo_palette;

        /* eink_3in27 self setup */
        par = info->par;
        par->fbinfo = info;
        par->spi = spi;
        par->dev = dev;
        par->display = &display;
        spi_set_drvdata(spi, par);

        tasklet_init(&par->task, eink_3in27_flush_task, (unsigned long)par);

        spin_lock_init(&par->dirty_lock);
        init_completion(&par->complete);
        eink_3in27_of_config(par);
        eink_3in27_hw_init(par);

        /* framebuffer register */
        // rc = register_framebuffer(info);
        // if (rc < 0) {
        //         dev_err(dev, "framebuffer register failed with %d!\n", rc);
        //         goto alloc_fail;
        // }

        return 0;

alloc_fail:
        vfree(vmem);

        return 0;
}

static int eink_3in27_remove(struct spi_device *spi)
{
        struct eink_3in27_par *par = spi_get_drvdata(spi);

        printk("%s\n", __func__);
        // kfree(par->buf);
        // kfree(par->txbuf.buf);
        // unregister_framebuffer(par->fbinfo);
        // framebuffer_release(par->fbinfo);
        return 0;
}

static const struct of_device_id eink_3in27_dt_ids[] = {
        { .compatible = "ultrachip,uc8253" },
        { /* KEEP THIS */ },
};
MODULE_DEVICE_TABLE(of, eink_3in27_dt_ids);

static const struct spi_device_id eink_3in27_spi_ids[] = {
        { "uc8253" },
        { /* KEEP THIS */ },
};
MODULE_DEVICE_TABLE(spi, eink_3in27_spi_ids);

static struct spi_driver eink_3in27_drv = {
        .driver = {
                .name = DRV_NAME,
                .of_match_table = of_match_ptr(eink_3in27_dt_ids),
        },
        .id_table = eink_3in27_spi_ids,
        .probe = eink_3in27_probe,
        .remove = eink_3in27_remove,
};

module_spi_driver(eink_3in27_drv);

MODULE_AUTHOR("Iota Hydrae <writeforever@foxmail.com>");
MODULE_DESCRIPTION("eink_3in27 E-Paper display framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:eink_3in27");
