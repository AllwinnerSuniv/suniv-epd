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

struct uc8253_par;

struct uc8253_operations {
        int (*refresh)(struct uc8253_par *ud, u8 mode);
        int (*reset)(struct uc8253_par *ud);
        int (*clear)(struct uc8253_par *ud);
        int (*sleep)(struct uc8253_par *ud);
        int (*is_busy)(struct uc8253_par *ud);
};

struct uc8253_display {
        u32                     xres;
        u32                     yres;
        u32                     bpp;
};

struct uc8253_par {

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
        
        struct uc8253_operations        ops;
        const struct uc8253_display           *display;
        
        struct fb_info          *fbinfo;
        struct fb_ops           *fbops;
        
        u32             palette_buffer[256];
        u32             palette_pal[16];
};

static int g_epd_3in52_flag = 0;
u32 pseudo_palette[16];

static int fbtft_write_spi(struct uc8253_par *ud, void *buf, size_t len)
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

static int fbtft_write_buf_dc(struct uc8253_par *ud, void *buf, size_t len, int dc)
{
        int rc;
        
        gpiod_set_value(ud->gpio.dc, dc);
        
        rc = fbtft_write_spi(ud, buf, len);
        if (rc < 0)
                dev_err(ud->dev, "write() failed and returned %d\n", rc);
                
        return rc;
}

static __inline int uc8253_send(struct uc8253_par *ud, u8 byte, int dc)
{
        fbtft_write_buf_dc(ud, &byte, 1, dc);
        return 0;
}
#define UC8253_DC_DATA          1
#define UC8253_DC_COMMAND       0
#define write_cmd(__ud, __c) uc8253_send(__ud, __c, UC8253_DC_COMMAND);
#define write_data(__ud, __d) uc8253_send(__ud, __d, UC8253_DC_DATA);

static int uc8253_init_display(struct uc8253_par *ud)
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

static int uc8253_load_lut_gc(struct uc8253_par *ud)
{
        int i;
        
        uc8253_load_loop(ud, 0x20, epd_3in52_lut_r20_gc);
        uc8253_load_loop(ud, 0x21, epd_3in52_lut_r21_gc);
        uc8253_load_loop(ud, 0x22, epd_3in52_lut_r22_gc);
        uc8253_load_loop(ud, 0x23, epd_3in52_lut_r23_gc);
        uc8253_load_loop(ud, 0x24, epd_3in52_lut_r24_gc);
        
        // if (g_epd_3in52_flag == 0) {
        //         uc8253_load_loop(ud, 0x22, epd_3in52_lut_r22_gc);
        //         uc8253_load_loop(ud, 0x23, epd_3in52_lut_r23_gc);
        // } else {
        //         uc8253_load_loop(ud, 0x22, epd_3in52_lut_r23_gc);
        //         uc8253_load_loop(ud, 0x23, epd_3in52_lut_r22_gc);
        // }
        return 0;
}

static int uc8253_reset(struct uc8253_par *ud)
{
        gpiod_set_value_cansleep(ud->gpio.reset, 1);
        msleep(10);
        gpiod_set_value_cansleep(ud->gpio.reset, 0);
        msleep(200);
        gpiod_set_value_cansleep(ud->gpio.reset, 1);
        msleep(10);
        
        return 0;
}

static int uc8253_refresh(struct uc8253_par *ud, u8 mode)
{
        write_cmd(ud, 0x17);
        write_data(ud, 0xa5);
        mdelay(200);
        return 0;
}

static int uc8253_clear(struct uc8253_par *ud)
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



static const int uc8253_show_img(struct uc8253_par *ud, const u8 *img)
{
        int i;
        write_cmd(ud, 0x13);
        for (i = 0; i < 360 * 240 / 8; i++)
                write_data(ud, img[i]);
                
        uc8253_load_lut_gc(ud);
        uc8253_refresh(ud, 0);
        return 0;
}

static int uc8253_request_one_gpio(struct uc8253_par *ud,
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

static int uc8253_request_gpios(struct uc8253_par *ud)
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

static int uc8253_of_config(struct uc8253_par *ud)
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

static int uc8253_hw_init(struct uc8253_par *ud)
{
        printk("%s, Display Panel initializing ...\n", __func__);
        uc8253_reset(ud);
        uc8253_init_display(ud);
        // uc8253_set_addr_win(ud, 0, 0, 200, 200);
        uc8253_clear(ud);
        
        uc8253_show_img(ud, internal_bootup_logo);
        
        uc8253_show_img(ud, internal_bootup_logo);
        
        return 0;
}

static const struct uc8253_display display = {
        .xres = 360,
        .yres = 240,
        .bpp = 1,
};

static irqreturn_t panel_busy_handle(int irq, void *data)
{
        return IRQ_HANDLED;
}

static int uc8253_setup_irq(struct uc8253_par *ud)
{
        struct device *dev = ud->dev;
        struct gpio_desc *busy;
        
        busy = devm_gpiod_get_optional(dev, "busy", GPIOD_IN);
        if (IS_ERR(busy))
                return dev_err_probe(dev, PTR_ERR(busy), "Failed to request the busy GPIO\n");
                
        if (!busy)
                return 0;

        return 0;
}

schedule_work(struct work_struct *work)

static void update_display(struct uc8253_par *ud)
{
        u8 *buf = ud->fbinfo->screen_buffer;
        
        // printk("%s\n", __func__);
        dev_dbg(ud->dev, "%s\n", __func__);
        /* write vmem to display then call refresh routine */
        /*
         * when this was called, driver should wait for busy pin comes low
         * until next frame refreshed
         */
        uc8253_show_img(ud, buf);
        // mdelay(1500);
}

static void uc8253_mkdirty(struct fb_info *info, int y, int height)
{
        struct uc8253_par *ud = info->par;
        // struct fb_deferred_io *fbdefio = info->fbdefio;
        dev_dbg(info->dev, "%s\n", __func__);
        // spin_lock(&ud->dirty_lock);
        // spin_unlock(&ud->dirty_lock);
        
        // schedule_delayed_work(&info->deferred_work, 0);
        update_display(ud);
}

static void uc8253_deferred_io(struct fb_info *info, struct list_head *pagelist)
{
        struct uc8253_par *ud = info->par;
        
        dev_dbg(info->dev, "%s\n", __func__);
        spin_lock(&ud->dirty_lock);
        spin_unlock(&ud->dirty_lock);
        update_display(info->par);
}

static ssize_t uc8253_fb_write(struct fb_info *info, const char __user *buf,
                               size_t count, loff_t *ppos)
{
        ssize_t res;
        dev_dbg(info->dev,
                "%s: count=%zd, ppos=%llu\n", __func__,  count, *ppos);
                
        // res = fb_sys_write(info, buf, count, ppos);
        // uc8253_mkdirty(info, -1, 0);
        return 0;
}

static void uc8253_fb_fillrect(struct fb_info *info,
                               const struct fb_fillrect *rect)
{
        dev_dbg(info->dev,
                "%s: dx=%d, dy=%d, width=%d, height=%d\n",
                __func__, rect->dx, rect->dy, rect->width, rect->height);
                
        sys_fillrect(info, rect);
        uc8253_mkdirty(info, rect->dy, rect->height);
}

static void uc8253_fb_copyarea(struct fb_info *info,
                               const struct fb_copyarea *area)
{
        dev_dbg(info->dev,
                "%s: dx=%d, dy=%d, width=%d, height=%d\n",
                __func__,  area->dx, area->dy, area->width, area->height);
                
        sys_copyarea(info, area);
        uc8253_mkdirty(info, area->dy, area->height);
}

static void uc8253_fb_imageblit(struct fb_info *info,
                                const struct fb_image *image)
{
        dev_dbg(info->dev,
                "%s: dx=%d, dy=%d, width=%d, height=%d\n",
                __func__,  image->dx, image->dy, image->width, image->height);
        sys_imageblit(info, image);
        
        uc8253_mkdirty(info, image->dy, image->height);
}

static int uc8253_fb_setcolreg(unsigned int regno, unsigned int red,
                               unsigned int green, unsigned int blue,
                               unsigned int transp, struct fb_info *info)
{
        int ret = 1;
        u8 val;
        
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

static int uc8253_probe(struct spi_device *spi)
{
        struct device *dev = &spi->dev;
        struct uc8253_par *ud;
        struct fb_deferred_io *fbdefio;
        struct fb_info *info;
        struct fb_ops *fbops;
        u8 *vmem = NULL;
        int vmem_size;
        int rc;
        
        printk("%s\n", __func__);
        /* memory resource alloc */
        // display = kmalloc(sizeof(struct uc8253_display), GFP_KERNEL);
        // if (!display) {
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
        info = framebuffer_alloc(sizeof(struct uc8253_par), dev);
        if (!info) {
                dev_err(dev, "failed to alloc framebuffer!\n");
                return -ENOMEM;
        }
        
        info->screen_buffer = vmem;
        info->fbops = fbops;
        info->fbdefio = fbdefio;
        
        fbops->owner = dev->driver->owner;
        fbops->fb_read = fb_sys_read;
        fbops->fb_write = uc8253_fb_write;
        fbops->fb_fillrect = uc8253_fb_fillrect;
        fbops->fb_copyarea = uc8253_fb_copyarea;
        fbops->fb_imageblit = uc8253_fb_imageblit;
        // fbops->fb_cursor = NULL;
        fbops->fb_setcolreg = uc8253_fb_setcolreg;
        // fbops->fb_blank = NULL;
        
        // fbdefio->delay = HZ;
        // fbdefio->deferred_io = uc8253_deferred_io;
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
        
        /* UC8253 self setup */
        ud = info->par;
        ud->fbinfo = info;
        ud->spi = spi;
        ud->dev = dev;
        ud->display = &display;
        spi_set_drvdata(spi, ud);
        
        spin_lock_init(&ud->dirty_lock);
        init_completion(&ud->complete);
        uc8253_of_config(ud);
        uc8253_hw_init(ud);
        
        /* framebuffer register */
        rc = register_framebuffer(info);
        if (rc < 0) {
                dev_err(dev, "framebuffer register failed with %d!\n", rc);
                goto alloc_fail;
        }
        
alloc_fail:
        vfree(vmem);
        
        return 0;
}

static int uc8253_remove(struct spi_device *spi)
{
        struct uc8253_par *ud = spi_get_drvdata(spi);
        
        printk("%s\n", __func__);
        // kfree(ud->buf);
        // kfree(ud->txbuf.buf);
        unregister_framebuffer(ud->fbinfo);
        framebuffer_release(ud->fbinfo);
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
MODULE_DESCRIPTION("UC8253 E-Paper display framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:uc8253");
