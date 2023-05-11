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

#define DRV_NAME "st7789v_drv"

struct st7789v_par;

struct st7789v_operations {
        int (*refresh)(struct st7789v_par *par, u8 mode);
        int (*reset)(struct st7789v_par *par);
        int (*clear)(struct st7789v_par *par);
        int (*sleep)(struct st7789v_par *par);
        int (*is_busy)(struct st7789v_par *par);
};

struct st7789v_display {
        u32                     xres;
        u32                     yres;
        u32                     bpp;
};

#define SUN6I_FIFO_DEPTH 128
struct st7789v_par {

        struct device           *dev;
        struct spi_device       *spi;
        u8                      *buf;
        // struct {
        //         void *buf;
        //         size_t len;
        // } txbuf;
        struct {
                struct gpio_desc *reset;
                struct gpio_desc *dc;
                struct gpio_desc *cs;
                struct gpio_desc *blk;
        } gpio;

        spinlock_t              dirty_lock;
        struct completion       complete;

        /* device specific */
        u32                     refr_mode;
        u32                     wait;
        u32                     busy;

        struct st7789v_operations        ops;
        const struct st7789v_display           *display;

        struct tasklet_struct task;

        struct fb_info          *fbinfo;
        struct fb_ops           *fbops;

        u32             palette_buffer[256];
        u32             pseudo_palette[16];
        u8 txbuf[SUN6I_FIFO_DEPTH];
};

// static int g_epd_3in27_flag = 0;

static int fbtft_write_spi(struct st7789v_par *par, void *buf, size_t len)
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

static int fbtft_write_buf_dc(struct st7789v_par *par, void *buf, size_t len, int dc)
{
        int rc;

        gpiod_set_value(par->gpio.dc, dc);

        rc = fbtft_write_spi(par, buf, len);
        if (rc < 0)
                dev_err(par->dev, "write() failed and returned %d\n", rc);

        return rc;
}

static __inline int st7789v_send(struct st7789v_par *par, u8 byte, int dc)
{
        fbtft_write_buf_dc(par, &byte, 1, dc);
        return 0;
}
#define st7789v_DC_DATA          1
#define st7789v_DC_COMMAND       0
#define write_cmd(__par, __c) st7789v_send(__par, __c, st7789v_DC_COMMAND);
#define write_data(__par, __d) st7789v_send(__par, __d, st7789v_DC_DATA);

static int st7789v_reset(struct st7789v_par *par)
{
        gpiod_set_value_cansleep(par->gpio.reset, 1);
        msleep(10);
        gpiod_set_value_cansleep(par->gpio.reset, 0);
        msleep(100);
        gpiod_set_value_cansleep(par->gpio.reset, 1);
        msleep(10);
        return 0;
}

static int st7789v_init_display(struct st7789v_par *par)
{
        st7789v_reset(par);
        mdelay(20);

        write_cmd(par, 0x11);
        mdelay(120);

        write_cmd(par,  0x36);
        write_data(par,  0x00);

        write_cmd(par,  0x3A);
        write_data(par,  0x05); /* RGB 565 */

        write_cmd(par,  0xB2);
        write_data(par,  0x0C);
        write_data(par,  0x0C);
        write_data(par,  0x00);
        write_data(par,  0x33);
        write_data(par,  0x33);

        write_cmd(par,  0xB7);
        write_data(par,  0x35);

        write_cmd(par,  0xBB);
        write_data(par,  0x32);

        write_cmd(par,  0xC2);
        write_data(par,  0x01);

        write_cmd(par,  0xC3);
        write_data(par,  0x15);

        write_cmd(par,  0xC4);
        write_data(par,  0x20);   //VDV, 0x20:0v

        write_cmd(par,  0xC6);
        write_data(par,  0x0F);   //0x13:60Hz

        write_cmd(par,  0xD0);
        write_data(par,  0xA4);
        write_data(par,  0xA1);

        write_cmd(par, 0xE0)
        write_data(par, 0xD0);
        write_data(par, 0x08);
        write_data(par, 0x0E);
        write_data(par, 0x09);
        write_data(par, 0x09);
        write_data(par, 0x05);
        write_data(par, 0x31);
        write_data(par, 0x33);
        write_data(par, 0x48);
        write_data(par, 0x17);
        write_data(par, 0x14);
        write_data(par, 0x15);
        write_data(par, 0x31);
        write_data(par, 0x34);


        write_cmd(par, 0xE1);
        write_data(par, 0xD0);
        write_data(par, 0x08);
        write_data(par, 0x0E);
        write_data(par, 0x09);
        write_data(par, 0x09);
        write_data(par, 0x15);
        write_data(par, 0x31);
        write_data(par, 0x33);
        write_data(par, 0x48);
        write_data(par, 0x17);
        write_data(par, 0x14);
        write_data(par, 0x15);
        write_data(par, 0x31);
        write_data(par, 0x34);

        write_cmd(par, 0x21);
        write_cmd(par, 0x29);

        return 0;
}

static int st7789v_blank(struct st7789v_par *par, bool on)
{
        if (on) {
                write_cmd(par, MIPI_DCS_SET_DISPLAY_OFF);
        } else {
                write_cmd(par, MIPI_DCS_SET_DISPLAY_ON);
        }
        return 0;
}

static void st7789v_set_addr_win(struct st7789v_par *par, int xs, int ys, int xe,
                                 int ye)
{
        // xs = xs + 20;
        // xe = xe + 20;
        write_cmd(par, MIPI_DCS_SET_COLUMN_ADDRESS);
        write_data(par, (xs >> BITS_PER_BYTE) & 0xff);
        write_data(par, (xs & 0xff));
        write_data(par, (xe >> BITS_PER_BYTE) & 0xff);
        write_data(par, (xe & 0xff));  /* 239 */

        write_cmd(par, MIPI_DCS_SET_PAGE_ADDRESS);
        write_data(par, (ys >> BITS_PER_BYTE) & 0xff);
        write_data(par, (ys & 0xff));
        write_data(par, (ye >> BITS_PER_BYTE) & 0xff);
        write_data(par, (ye & 0xff));  /* 279 */

        write_cmd(par, MIPI_DCS_WRITE_MEMORY_START);
}

static int st7789v_sleep(struct st7789v_par *par)
{
        return 0;
}

static int st7789v_clear(struct st7789v_par *par)
{
        int i;
        st7789v_set_addr_win(par, 0, 0, 240, 320);
        memset(par->txbuf, 0x78, 128);

        for (i = 0; i < (240 * 280 * 2) / 128; i++) {
                fbtft_write_buf_dc(par, par->txbuf, 128, 1);
        }
        return 0;
}

static const struct st7789v_operations default_st7789v_ops = {
        .clear = st7789v_clear,
        .reset = NULL,
        .is_busy = NULL,
        .sleep = NULL,
};

static const int st7789v_show_img(struct st7789v_par *par, const u8 *img)
{
        int i;

        return 0;
}

static int st7789v_request_one_gpio(struct st7789v_par *par,
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

static int st7789v_request_gpios(struct st7789v_par *par)
{
        int rc;
        pr_debug("%s, configure from dt\n", __func__);

        rc = st7789v_request_one_gpio(par, "reset-gpios", 0, &par->gpio.reset);
        if (rc)
                return rc;
        rc = st7789v_request_one_gpio(par, "dc-gpios", 0, &par->gpio.dc);
        if (rc)
                return rc;
        rc = st7789v_request_one_gpio(par, "blk-gpios", 0, &par->gpio.blk);
        if (rc)
                return rc;
        rc = st7789v_request_one_gpio(par, "cs-gpios", 0, &par->gpio.cs);
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

static int st7789v_of_config(struct st7789v_par *par)
{
        int rc;

        printk("%s\n", __func__);
        rc = st7789v_request_gpios(par);
        if (rc) {
                dev_err(par->dev, "Request gpios failed!\n");
                return rc;
        }
        return 0;

        /* request xres and yres from dt */
}

static int st7789v_hw_init(struct st7789v_par *par)
{
        printk("%s, Display Panel initializing ...\n", __func__);
        st7789v_init_display(par);
        st7789v_clear(par);

        // tasklet_enable(&par->task);
        return 0;
}

static const struct st7789v_display display = {
        .xres = 240,
        .yres = 280,
        .bpp = 16,
};

static void write_vmem(struct st7789v_par *par, size_t offset, size_t len)
{
        dev_dbg(par->dev, "%s\n", __func__);
        u16 *vmem16;
        size_t remain;

        remain = len /2;
        vmem16 = (u16 *)(par->fbinfo->screen_buffer + offset);

        gpiod_set_value(par->gpio.dc, 1);

        /* non-buffered spi write */
        fbtft_write_spi(par, vmem16, len);
}

static void update_display(struct st7789v_par *par, unsigned int start_line,
                        unsigned int end_line)
{
        u8 *buf = par->fbinfo->screen_buffer;
        size_t offset, len;
        int i;

        // printk("%s\n", __func__);
        dev_dbg(par->dev, "%s\n", __func__);
        /* write vmem to display then call refresh routine */
        /*
         * when this was called, driver should wait for busy pin comes low
         * until next frame refreshed
         */
        start_line = 0;
        end_line = par->fbinfo->var.yres-1;

        st7789v_set_addr_win(par, 0, start_line, par->fbinfo->var.xres - 1, end_line);

        offset = start_line * par->fbinfo->fix.line_length;
        len = (end_line - start_line + 1) * par->fbinfo->fix.line_length;

        write_vmem(par, offset, len);
}

// static void st7789v_flush_task(unsigned long data)
// {
//         // int timeout;
//         struct st7789v_par *par = (struct st7789v_par *)data;
// }

static void st7789v_mkdirty(struct fb_info *info, int y, int height)
{
        struct st7789v_par *par = info->par;
        struct fb_deferred_io *fbdefio = info->fbdefio;

        dev_dbg(info->dev, "%s\n", __func__);

        if (y == -1) {
                y = 0;
                height = info->var.yres;
        }

        /* mark dirty lines here, but update all for now */
        // spin_lock(&par->dirty_lock);
        // spin_unlock(&par->dirty_lock);

        schedule_delayed_work(&info->deferred_work, fbdefio->delay);
}

static void st7789v_deferred_io(struct fb_info *info, struct list_head *pagelist)
{
        struct st7789v_par *par = info->par;
        unsigned int dirty_lines_start, dirty_lines_end;
        unsigned int y_low = 0, y_high = 0;
        unsigned long index;
        struct page *page;
        int count = 0;

        list_for_each_entry(page, pagelist, lru) {
                count++;
                index = page->index << PAGE_SHIFT;
                y_low = index / info->fix.line_length;
                y_high = (index + PAGE_SIZE - 1) / info->fix.line_length;
                dev_dbg(info->device,
                        "page->index=%lu y_low=%d y_high=%d\n",
                        page->index, y_low, y_high);
                if (y_high > info->var.yres - 1)
                        y_high = info->var.yres - 1;
                if (y_low < dirty_lines_start)
                        dirty_lines_start = y_low;
                if (y_high > dirty_lines_end)
                        dirty_lines_end = y_high;
        }

        update_display(par, dirty_lines_start, dirty_lines_end);
}

static void st7789v_fb_fillrect(struct fb_info *info,
                                const struct fb_fillrect *rect)
{
        dev_dbg(info->dev,
                "%s: dx=%d, dy=%d, width=%d, height=%d\n",
                __func__, rect->dx, rect->dy, rect->width, rect->height);

        sys_fillrect(info, rect);
        st7789v_mkdirty(info, rect->dy, rect->height);
}

static void st7789v_fb_copyarea(struct fb_info *info,
                                const struct fb_copyarea *area)
{
        dev_dbg(info->dev,
                "%s: dx=%d, dy=%d, width=%d, height=%d\n",
                __func__,  area->dx, area->dy, area->width, area->height);

        sys_copyarea(info, area);
        st7789v_mkdirty(info, area->dy, area->height);
}

static void st7789v_fb_imageblit(struct fb_info *info,
                                 const struct fb_image *image)
{
        dev_dbg(info->dev,
                "%s: dx=%d, dy=%d, width=%d, height=%d\n",
                __func__,  image->dx, image->dy, image->width, image->height);
        sys_imageblit(info, image);

        st7789v_mkdirty(info, image->dy, image->height);
}

static ssize_t st7789v_fb_write(struct fb_info *info, const char __user *buf,
                                size_t count, loff_t *ppos)
{
        ssize_t res;
        dev_dbg(info->dev,
                "%s: count=%zd, ppos=%llu\n", __func__,  count, *ppos);

        res = fb_sys_write(info, buf, count, ppos);

        st7789v_mkdirty(info, -1, 0);
        return 0;
}

static unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
        chan &= 0xfff;
        chan >>= 16 - bf->length;
        return chan << bf->offset;
}

static int st7789v_fb_setcolreg(unsigned int regno, unsigned int red,
                                unsigned int green, unsigned int blue,
                                unsigned int transp, struct fb_info *info)
{
        int ret = 1;
        unsigned int val;

        dev_dbg(info->dev,
                "%s(regno=%u, red=0x%X, green=0x%X, blue=0x%X, trans=0x%X)\n",
                __func__, regno, red, green, blue, transp);

        switch (info->fix.visual) {
        case FB_VISUAL_TRUECOLOR:
                if (regno < 16) {
                        u32 *pal = info->pseudo_palette;

                        val  = chan_to_field(red, &info->var.red);
                        val |= chan_to_field(green, &info->var.green);
                        val += chan_to_field(blue, &info->var.blue);

                        pal[regno] = val;
                        ret = 0;
                }
                break;
        case FB_VISUAL_MONO01:
                dev_dbg(info->dev, "FB_VISUAL_MONO01\n");
                break;
        }

        return ret;
}

static int st7789v_fb_blank(int blank, struct fb_info *info)
{
        struct st7789v_par *par = info->par;
        int ret = -EINVAL;

        switch (blank) {
        case FB_BLANK_POWERDOWN:
        case FB_BLANK_VSYNC_SUSPEND:
        case FB_BLANK_HSYNC_SUSPEND:
        case FB_BLANK_NORMAL:
                ret = st7789v_blank(par, true);
                break;
        case FB_BLANK_UNBLANK:
                ret = st7789v_blank(par, false);
                break;
        }
        return ret;
}

static int st7789v_probe(struct spi_device *spi)
{
        struct device *dev = &spi->dev;
        struct st7789v_par *par;
        struct fb_deferred_io *fbdefio;
        struct fb_info *info;
        struct fb_ops *fbops;
        u8 *vmem = NULL;
        int vmem_size;
        int rc;

        printk("%s\n", __func__);
        /* memory resource alloc */
        // display = kmalloc(sizeof(struct st7789v_display), GFP_KERNEL);
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
        printk("vmem_size : %d\n", vmem_size);
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
        info = framebuffer_alloc(sizeof(struct st7789v_par), dev);
        if (!info) {
                dev_err(dev, "failed to alloc framebuffer!\n");
                return -ENOMEM;
        }

        info->screen_buffer = vmem;
        info->fbops = fbops;
        info->fbdefio = fbdefio;

        fbops->owner = dev->driver->owner;
        fbops->fb_read = fb_sys_read;
        fbops->fb_write = st7789v_fb_write;
        fbops->fb_fillrect = st7789v_fb_fillrect;
        fbops->fb_copyarea = st7789v_fb_copyarea;
        fbops->fb_imageblit = st7789v_fb_imageblit;
        fbops->fb_setcolreg = st7789v_fb_setcolreg;
        fbops->fb_blank = st7789v_fb_blank;
        // fbops->fb_cursor = NULL;

        fbdefio->delay = HZ / 30;
        fbdefio->deferred_io = st7789v_deferred_io;
        fb_deferred_io_init(info);

        snprintf(info->fix.id, sizeof(info->fix.id), "%s", dev->driver->name);
        info->fix.type            =       FB_TYPE_PACKED_PIXELS;
        info->fix.visual          =       FB_VISUAL_TRUECOLOR;
        info->fix.xpanstep        =       0;
        info->fix.ypanstep        =       0;
        info->fix.ywrapstep       =       0;
        info->fix.line_length     =       display.xres * display.bpp / BITS_PER_BYTE;
        info->fix.accel           =       FB_ACCEL_NONE;
        info->fix.smem_len        =       vmem_size;

        info->var.rotate          =       0;
        info->var.xres            =       display.xres;
        info->var.yres            =       display.yres;
        info->var.xres_virtual    =       info->var.xres;
        info->var.yres_virtual    =       info->var.yres;
        info->var.bits_per_pixel  =       display.bpp;
        info->var.nonstd          =       1;

        info->var.red.offset      =       11;
        info->var.red.length      =       5;
        info->var.green.offset    =       5;
        info->var.green.length    =       6;
        info->var.blue.offset     =       0;
        info->var.blue.length     =       5;
        info->var.transp.offset   =       0;
        info->var.transp.length   =       0;

        info->var.grayscale     = 0;

        info->flags = FBINFO_FLAG_DEFAULT | FBINFO_VIRTFB;

        /* st7789v self setup */
        par = info->par;
        info->pseudo_palette = &par->pseudo_palette;

        par->fbinfo = info;
        par->spi = spi;
        par->dev = dev;
        par->buf = par->txbuf;
        par->display = &display;
        spi_set_drvdata(spi, par);

        // tasklet_init(&par->task, st7789v_flush_task, (unsigned long)par);

        spin_lock_init(&par->dirty_lock);
        init_completion(&par->complete);
        st7789v_of_config(par);
        st7789v_hw_init(par);

        /* framebuffer register */
        rc = register_framebuffer(info);
        if (rc < 0) {
                dev_err(dev, "framebuffer register failed with %d!\n", rc);
                goto alloc_fail;
        }

        return 0;

alloc_fail:
        vfree(vmem);

        return 0;
}

static int st7789v_remove(struct spi_device *spi)
{
        struct st7789v_par *par = spi_get_drvdata(spi);

        printk("%s\n", __func__);
        // kfree(par->buf);
        // kfree(par->txbuf.buf);
        unregister_framebuffer(par->fbinfo);
        framebuffer_release(par->fbinfo);
        return 0;
}

static const struct of_device_id st7789v_dt_ids[] = {
        { .compatible = "ultrachip,uc8253" },
        { /* KEEP THIS */ },
};
MODULE_DEVICE_TABLE(of, st7789v_dt_ids);

static const struct spi_device_id st7789v_spi_ids[] = {
        { "uc8253" },
        { /* KEEP THIS */ },
};
MODULE_DEVICE_TABLE(spi, st7789v_spi_ids);

static struct spi_driver st7789v_drv = {
        .driver = {
                .name = DRV_NAME,
                .of_match_table = of_match_ptr(st7789v_dt_ids),
        },
        .id_table = st7789v_spi_ids,
        .probe = st7789v_probe,
        .remove = st7789v_remove,
};

module_spi_driver(st7789v_drv);

MODULE_AUTHOR("Iota Hydrae <writeforever@foxmail.com>");
MODULE_DESCRIPTION("st7789v E-Paper display framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:st7789v");
