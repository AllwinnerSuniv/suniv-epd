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

#define DRV_NAME "ssd1681_drv"

struct ssd1681_par;

struct ssd1681_operations {
        int (*reset)(struct ssd1681_par *par);
        int (*clear)(struct ssd1681_par *par);
        int (*idle)(struct ssd1681_par *par, bool on);
        int (*blank)(struct ssd1681_par *par, bool on);
        int (*sleep)(struct ssd1681_par *par, bool on);
        int (*set_addr_win)(struct ssd1681_par *par, int xs, int ys, int xe, int ye);
};

struct ssd1681_display {
        u32                     xres;
        u32                     yres;
        u32                     bpp;
        u32                     fps;
        u32                     rotate;
        u32                     xs_off;
        u32                     xe_off;
        u32                     ys_off;
        u32                     ye_off;
};

#define SUNIV_FIFO_DEPTH 128
struct ssd1681_par {

        struct device           *dev;
        struct spi_device       *spi;
        u8                      *buf;
        struct {
                void *buf;
                size_t len;
        } txbuf;
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

        const struct ssd1681_operations        *tftops;
        const struct ssd1681_display           *display;

        struct tasklet_struct task;

        struct fb_info          *fbinfo;
        struct fb_ops           *fbops;

        u32             pseudo_palette[16];

        u32             dirty_lines_start;
        u32             dirty_lines_end;

};
// u8 txbuf[SUNIV_FIFO_DEPTH];
// static int g_epd_3in27_flag = 0;

static int fbtft_write_spi(struct ssd1681_par *par, void *buf, size_t len)
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

static int fbtft_write_buf_dc(struct ssd1681_par *par, void *buf, size_t len, int dc)
{
        int rc;

        gpiod_set_value(par->gpio.dc, dc);

        rc = fbtft_write_spi(par, buf, len);
        if (rc < 0)
                dev_err(par->dev, "write() failed and returned %d\n", rc);

        return rc;
}

// static __inline int ssd1681_send(struct ssd1681_par *par, u8 byte, int dc)
// {
//         fbtft_write_buf_dc(par, &byte, 1, dc);
//         return 0;
// }
// #define ssd1681_DC_DATA          1
// #define ssd1681_DC_COMMAND       0
// #define write_cmd(__par, __c) ssd1681_send(__par, __c, ssd1681_DC_COMMAND);
// #define write_data(__par, __d) ssd1681_send(__par, __d, ssd1681_DC_DATA);

#define NUMARGS(...)  (sizeof((int[]){__VA_ARGS__}) / sizeof(int))
static int __maybe_unused ssd1681_write_reg(struct ssd1681_par *par, int len, ...)
{
        u8 *buf = (u8 *)par->buf;
        va_list args;
        int i;

        va_start(args, len);

        *buf = (u8)va_arg(args, unsigned int);
        fbtft_write_buf_dc(par, buf, sizeof(u8), 0);
        len--;

        /* if there no params */
        if (len == 0)
                return 0;

        for (i = 0; i < len; i++)
                *buf++ = (u8)va_arg(args, unsigned int);

        fbtft_write_buf_dc(par, par->buf, len, 1);
        va_end(args);

        return 0;
}
#define write_reg(par, ...) \
        ssd1681_write_reg(par, NUMARGS(__VA_ARGS__), __VA_ARGS__)

static int ssd1681_reset(struct ssd1681_par *par)
{
        gpiod_set_value(par->gpio.reset, 1);
        gpiod_set_value(par->gpio.reset, 0);
        msleep(2);
        gpiod_set_value(par->gpio.reset, 1);
        return 0;
}

static int ssd1681_init_display(struct ssd1681_par *par)
{
        printk("%s\n", __func__);
        ssd1681_reset(par);

        write_reg(par, 0x12);
        msleep(100);

        write_reg(par, 0x01, 0xc7, 0x00, 0x01);

        write_reg(par, 0x11, 0x01);

        write_reg(par, 0x44, 0x00, 0x18);

        write_reg(par, 0x45, 0xc7, 0x00, 0x00, 0x00);

        write_reg(par, 0x3c, 0x05);

        write_reg(par, 0x18, 0x80);

        write_reg(par, 0x4e, 0x4f, 0xc7, 0x00);
        msleep(100);

        write_reg(par, 0xe2, 0xff);

        return 0;
}

static int ssd1681_blank(struct ssd1681_par *par, bool on)
{
        return 0;
}

static int ssd1681_set_win(struct ssd1681_par *par, int xs, int ys, int xe,
                           int ye)
{
        write_reg(par, 0x44, (xs >> 3) & 0xFF, (xe >> 3) & 0xFF);

        /* set start/end position address of y in ram */
        write_reg(par, 0x45, ys & 0xFF, (ys >> 8) & 0xFF, ye & 0xFF, (ye >> 8) & 0xFF);
        return 0;
}

static int ssd1681_set_cursor(struct ssd1681_par *par, int x, int y)
{
        /* set address counter of x in ram */
        write_reg(par,  0x4E, (x >> 3) & 0xFF);

        /* set address counter of y in ram */
        write_reg(par,  0x4F, y & 0xFF, (y >> 8) & 0xFF);

        return 0;
}

static int ssd1681_set_addr_win(struct ssd1681_par *par, int xs, int ys, int xe,
                                int ye)
{
        write_reg(par, 0x44, (xs >> 3) & 0xff, (xe >> 3) & 0xff);
        write_reg(par, 0x45, ys & 0xff, (ys >> 8) & 0xff,
                  ye & 0xff, (ye >> 8) & 0xff);

        write_reg(par, 0x4e, (xs >> 3) & 0xff);
        write_reg(par, 0x4f, ys & 0xff, (ys >> 8) & 0xff);

        write_reg(par, 0xe4);
        return 0;
}

static int ssd1681_idle(struct ssd1681_par *par, bool on)
{
        return 0;
}

static int ssd1681_sleep(struct ssd1681_par *par, bool on)
{
        return 0;
}

static int __maybe_unused ssd1681_clear(struct ssd1681_par *par)
{
        int i, j;
        u8 data = 0x0, clear = 0xff;
        ssd1681_set_win(par, 0, 0, 200, 200);
        write_reg(par, 0x24);

        for (i = 0; i < 200; i++) {
                ssd1681_set_cursor(par, 0, i);
                for (j = 0; j < 25; j++)
                        fbtft_write_buf_dc(par, &data, 1, 1);
        }

        write_reg(par, 0xe2);
        write_reg(par, 0xff);
        write_reg(par, 0xe0);
        msleep(200);

        ssd1681_set_win(par, 0, 0, 200, 200);
        write_reg(par, 0x24);

        for (i = 0; i < 200; i++) {
                ssd1681_set_cursor(par, 0, i);
                for (j = 0; j < 25; j++)
                        fbtft_write_buf_dc(par, &data, 1, 1);
        }

        write_reg(par, 0xe2);
        write_reg(par, 0xff);
        write_reg(par, 0xe0);
        msleep(200);

        return 0;
}

static const struct ssd1681_operations default_ssd1681_ops = {
        .idle  = ssd1681_idle,
        .clear = ssd1681_clear,
        .blank = ssd1681_blank,
        .reset = ssd1681_reset,
        .sleep = ssd1681_sleep,
        .set_addr_win = ssd1681_set_addr_win,
};

static int ssd1681_request_one_gpio(struct ssd1681_par *par,
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

static int ssd1681_request_gpios(struct ssd1681_par *par)
{
        int rc;
        pr_debug("%s, configure from dt\n", __func__);

        rc = ssd1681_request_one_gpio(par, "reset-gpios", 0, &par->gpio.reset);
        if (rc)
                return rc;
        rc = ssd1681_request_one_gpio(par, "dc-gpios", 0, &par->gpio.dc);
        if (rc)
                return rc;
        rc = ssd1681_request_one_gpio(par, "blk-gpios", 0, &par->gpio.blk);
        if (rc)
                return rc;
        rc = ssd1681_request_one_gpio(par, "cs-gpios", 0, &par->gpio.cs);
        if (rc)
                return rc;

        return 0;
}

/* returns 0 if the property is not present */
static u32 __maybe_unused fbtft_property_value(struct device *dev, const char *propname)
{
        int ret;
        u32 val = 0;

        ret = device_property_read_u32(dev, propname, &val);
        if (ret == 0)
                dev_info(dev, "%s: %s = %u\n", __func__, propname, val);

        return val;
}

static int ssd1681_of_config(struct ssd1681_par *par)
{
        int rc;

        printk("%s\n", __func__);
        rc = ssd1681_request_gpios(par);
        if (rc) {
                dev_err(par->dev, "Request gpios failed!\n");
                return rc;
        }
        return 0;

        /* request xres and yres from dt */
}

static int ssd1681_set_var(struct ssd1681_par *par)
{
        return 0;
}

static int ssd1681_hw_init(struct ssd1681_par *par)
{
        printk("%s\n", __func__);
        ssd1681_init_display(par);
        ssd1681_set_var(par);
        ssd1681_clear(par);

        return 0;
}

/* TODO: device seems received wrong color format, check data transfer routine */
static int write_vmem(struct ssd1681_par *par, size_t offset, size_t len)
{
        u16 *vmem16;
        __be16 *txbuf16 = par->txbuf.buf;
        size_t remain;
        size_t to_copy;
        size_t tx_array_size;
        int i;

        dev_dbg(par->dev, "%s, offset = %d, len = %d\n", __func__, offset, len);

        remain = len / 2;
        vmem16 = (u16 *)(par->fbinfo->screen_buffer + offset);

        gpiod_set_value(par->gpio.dc, 1);

        /* non-buffered spi write */
        if (!par->txbuf.buf)
                return fbtft_write_spi(par, vmem16, len);

        tx_array_size = par->txbuf.len / 2;

        while (remain) {
                to_copy = min(tx_array_size, remain);
                dev_dbg(par->fbinfo->device, "to_copy=%zu, remain=%zu\n",
                        to_copy, remain - to_copy);

                for (i = 0; i < to_copy; i++)
                        txbuf16[i] = cpu_to_be16(vmem16[i]);

                vmem16 = vmem16 + to_copy;
                /* send batch to device */
                fbtft_write_spi(par, par->txbuf.buf,  to_copy * 2);

                remain -= to_copy;
        }
        return 0;
}

static void update_display(struct ssd1681_par *par, unsigned int start_line,
                           unsigned int end_line)
{
        size_t offset, len;

        dev_dbg(par->dev, "%s, start_line : %d, end_line : %d\n", __func__, start_line, end_line);

        // par->tftops->idle(par, false);
        /* write vmem to display then call refresh routine */
        /*
         * when this was called, driver should wait for busy pin comes low
         * until next frame refreshed
         */
        if (start_line > end_line) {
                dev_dbg(par->dev, "start line never should bigger than end line !!!!!\n");
                start_line = 0;
                end_line = par->fbinfo->var.yres - 1;
        }

        if (start_line > par->fbinfo->var.yres - 1 ||
            end_line > par->fbinfo->var.yres - 1) {
                dev_dbg(par->dev, "invaild start line or end line !!!!!\n");
                start_line = 0;
                end_line = par->fbinfo->var.yres - 1;
        }

        // start_line = 0;
        // end_line = par->fbinfo->var.yres - 1;

        /* for each column, refresh dirty rows */
        par->tftops->set_addr_win(par, 0, start_line, par->fbinfo->var.xres - 1, end_line);

        offset = start_line * par->fbinfo->fix.line_length;
        len = (end_line - start_line + 1) * par->fbinfo->fix.line_length;

        write_vmem(par, offset, len);

        // par->tftops->idle(par, true);
}

static void ssd1681_mkdirty(struct fb_info *info, int y, int height)
{
        struct ssd1681_par *par = info->par;
        struct fb_deferred_io *fbdefio = info->fbdefio;

        dev_dbg(info->dev, "%s, y : %d, height : %d\n", __func__, y, height);

        if (y == -1) {
                y = 0;
                height = info->var.yres;
        }

        /* mark dirty lines here, but update all for now */
        spin_lock(&par->dirty_lock);
        if (y < par->dirty_lines_start)
                par->dirty_lines_start = y;
        if (y + height - 1 > par->dirty_lines_end)
                par->dirty_lines_end = y + height - 1;
        spin_unlock(&par->dirty_lock);

        schedule_delayed_work(&info->deferred_work, fbdefio->delay);
}

static void ssd1681_deferred_io(struct fb_info *info, struct list_head *pagelist)
{
        struct ssd1681_par *par = info->par;
        unsigned int dirty_lines_start, dirty_lines_end;
        unsigned int y_low = 0, y_high = 0;
        unsigned long index;
        struct page *page;
        int count = 0;

        spin_lock(&par->dirty_lock);
        dirty_lines_start = par->dirty_lines_start;
        dirty_lines_end = par->dirty_lines_end;

        /* clean dirty markers */
        par->dirty_lines_start = par->fbinfo->var.yres - 1;
        par->dirty_lines_end = 0;
        spin_unlock(&par->dirty_lock);

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

        dev_dbg(info->device,
                "%s, dirty_line  start : %d, end : %d\n",
                __func__, dirty_lines_start, dirty_lines_end);
        update_display(par, dirty_lines_start, dirty_lines_end);
}

static void ssd1681_fb_fillrect(struct fb_info *info,
                                const struct fb_fillrect *rect)
{
        dev_dbg(info->dev,
                "%s: dx=%d, dy=%d, width=%d, height=%d\n",
                __func__, rect->dx, rect->dy, rect->width, rect->height);

        sys_fillrect(info, rect);
        ssd1681_mkdirty(info, rect->dy, rect->height);
}

static void ssd1681_fb_copyarea(struct fb_info *info,
                                const struct fb_copyarea *area)
{
        dev_dbg(info->dev,
                "%s: dx=%d, dy=%d, width=%d, height=%d\n",
                __func__,  area->dx, area->dy, area->width, area->height);

        sys_copyarea(info, area);
        ssd1681_mkdirty(info, area->dy, area->height);
}

static void ssd1681_fb_imageblit(struct fb_info *info,
                                 const struct fb_image *image)
{
        dev_dbg(info->dev,
                "%s: dx=%d, dy=%d, width=%d, height=%d\n",
                __func__,  image->dx, image->dy, image->width, image->height);
        sys_imageblit(info, image);

        ssd1681_mkdirty(info, image->dy, image->height);
}

static ssize_t ssd1681_fb_write(struct fb_info *info, const char __user *buf,
                                size_t count, loff_t *ppos)
{
        ssize_t res;
        dev_dbg(info->dev,
                "%s: count=%zd, ppos=%llu\n", __func__,  count, *ppos);

        res = fb_sys_write(info, buf, count, ppos);

        ssd1681_mkdirty(info, -1, 0);
        return 0;
}

/* from pxafb.c */
static unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
        chan &= 0xffff;
        chan >>= 16 - bf->length;
        return chan << bf->offset;
}

static int ssd1681_fb_setcolreg(unsigned int regno, unsigned int red,
                                unsigned int green, unsigned int blue,
                                unsigned int transp, struct fb_info *info)
{
        unsigned int val;
        int ret = 1;

        dev_dbg(info->dev,
                "%s(regno=%u, red=0x%X, green=0x%X, blue=0x%X, trans=0x%X)\n",
                __func__, regno, red, green, blue, transp);

        switch (info->fix.visual) {
        case FB_VISUAL_TRUECOLOR:
                if (regno < 16) {
                        u32 *pal = info->pseudo_palette;

                        val  = chan_to_field(red, &info->var.red);
                        val |= chan_to_field(green, &info->var.green);
                        val |= chan_to_field(blue, &info->var.blue);

                        pal[regno] = val;
                        ret = 0;
                }
                break;
        }

        return ret;
}

static int ssd1681_fb_blank(int blank, struct fb_info *info)
{
        struct ssd1681_par *par = info->par;
        int ret = -EINVAL;

        switch (blank) {
        case FB_BLANK_POWERDOWN:
        case FB_BLANK_VSYNC_SUSPEND:
        case FB_BLANK_HSYNC_SUSPEND:
        case FB_BLANK_NORMAL:
                ret = ssd1681_blank(par, true);
                break;
        case FB_BLANK_UNBLANK:
                ret = ssd1681_blank(par, false);
                break;
        }
        return ret;
}

static const struct ssd1681_display display = {
        .xres = 84,
        .yres = 48,
        .bpp = 1,
        .fps = 30,
        .rotate = 0,
};

static int ssd1681_probe(struct spi_device *spi)
{
        struct device *dev = &spi->dev;
        struct ssd1681_par *par;
        struct fb_deferred_io *fbdefio;
        int width, height, bpp, rotate;
        struct fb_info *info;
        struct fb_ops *fbops;
        u8 *vmem = NULL;
        int vmem_size;
        int rc;

        printk("%s\n", __func__);
        /* memory resource alloc */

        rotate = display.rotate;
        bpp = display.bpp;
        switch (rotate) {
        case 90:
        case 270:
                width = display.yres;
                height = display.xres;
                break;
        default:
                width = display.xres;
                height = display.yres;
                break;
        }

        vmem_size = (width * height * bpp) / BITS_PER_BYTE;
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
        info = framebuffer_alloc(sizeof(struct ssd1681_par), dev);
        if (!info) {
                dev_err(dev, "failed to alloc framebuffer!\n");
                return -ENOMEM;
        }

        info->screen_buffer = vmem;
        info->fbops = fbops;
        info->fbdefio = fbdefio;

        fbops->owner        = dev->driver->owner;
        fbops->fb_read      = fb_sys_read;
        fbops->fb_write     = ssd1681_fb_write;
        fbops->fb_fillrect  = ssd1681_fb_fillrect;
        fbops->fb_copyarea  = ssd1681_fb_copyarea;
        fbops->fb_imageblit = ssd1681_fb_imageblit;
        fbops->fb_setcolreg = ssd1681_fb_setcolreg;
        fbops->fb_blank     = ssd1681_fb_blank;

        snprintf(info->fix.id, sizeof(info->fix.id), "%s", dev->driver->name);
        info->fix.type            =       FB_TYPE_PACKED_PIXELS;
        info->fix.visual          =       FB_VISUAL_TRUECOLOR;
        info->fix.xpanstep        =       0;
        info->fix.ypanstep        =       0;
        info->fix.ywrapstep       =       0;
        info->fix.line_length     =       width * bpp / BITS_PER_BYTE;
        info->fix.accel           =       FB_ACCEL_NONE;
        info->fix.smem_len        =       vmem_size;

        info->var.rotate          =       rotate;
        info->var.xres            =       width;
        info->var.yres            =       height;
        info->var.xres_virtual    =       info->var.xres;
        info->var.yres_virtual    =       info->var.yres;
        info->var.bits_per_pixel  =       bpp;
        info->var.nonstd          =       1;

        info->var.red.offset      =       11;
        info->var.red.length      =       5;
        info->var.green.offset    =       5;
        info->var.green.length    =       6;
        info->var.blue.offset     =       0;
        info->var.blue.length     =       5;
        info->var.transp.offset   =       0;
        info->var.transp.length   =       0;

        // info->var.grayscale     = 0;
        info->flags = FBINFO_FLAG_DEFAULT | FBINFO_VIRTFB;

        fbdefio->delay = HZ / display.fps;
        fbdefio->deferred_io = ssd1681_deferred_io;
        fb_deferred_io_init(info);

        /* ssd1681 self setup */
        par = info->par;
        info->pseudo_palette = &par->pseudo_palette;

        par->fbinfo = info;
        par->spi = spi;
        par->dev = dev;

        par->buf = devm_kzalloc(dev, 128, GFP_KERNEL);
        if (!par->buf) {
                dev_err(dev, "failed to alloc buf memory!\n");
                return -ENOMEM;
        }

        par->txbuf.buf = devm_kzalloc(dev, vmem_size, GFP_KERNEL);
        if (!par->txbuf.buf) {
                dev_err(dev, "failed to alloc txbuf!\n");
                return -ENOMEM;
        }
        par->txbuf.len = vmem_size;

        par->tftops = &default_ssd1681_ops;
        par->display = &display;

        dev_set_drvdata(dev, par);
        spi_set_drvdata(spi, par);

        spin_lock_init(&par->dirty_lock);
        init_completion(&par->complete);
        ssd1681_of_config(par);
        ssd1681_hw_init(par);

        /* framebuffer register */
        // rc = register_framebuffer(info);
        // if (rc < 0) {
        //         dev_err(dev, "framebuffer register failed with %d!\n", rc);
        //         goto alloc_fail;
        // }

        printk("%zu KB buffer memory\n", par->txbuf.len >> 10);
        printk(" spi%d.%d at %d MHz\n", spi->master->bus_num, spi->chip_select,
               spi->max_speed_hz / 1000000);
        printk("%d KB video memory\n", info->fix.smem_len >> 10);

alloc_fail:
        vfree(vmem);

        return 0;
}

static int ssd1681_remove(struct spi_device *spi)
{
        struct ssd1681_par *par = spi_get_drvdata(spi);

        printk("%s\n", __func__);
        // fb_deferred_io_cleanup(par->fbinfo);

        // unregister_framebuffer(par->fbinfo);
        // framebuffer_release(par->fbinfo);
        return 0;
}

static int __maybe_unused ssd1681_runtime_suspend(struct device *dev)
{
        struct ssd1681_par *par = dev_get_drvdata(dev);

        par->tftops->sleep(par, true);

        return 0;
}

static int __maybe_unused ssd1681_runtime_resume(struct device *dev)
{
        struct ssd1681_par *par = dev_get_drvdata(dev);

        par->tftops->sleep(par, false);

        return 0;
}

static int __maybe_unused ssd1681_runtime_idle(struct device *dev)
{
        struct ssd1681_par *par = dev_get_drvdata(dev);

        par->tftops->idle(par, true);

        return 0;
}

static const struct of_device_id ssd1681_dt_ids[] = {
        { .compatible = "ultrachip,uc8253" },
        { /* KEEP THIS */ },
};
MODULE_DEVICE_TABLE(of, ssd1681_dt_ids);

static const struct spi_device_id ssd1681_spi_ids[] = {
        { "uc8253" },
        { /* KEEP THIS */ },
};
MODULE_DEVICE_TABLE(spi, ssd1681_spi_ids);

#if CONFIG_PM
static const struct dev_pm_ops ssd1681_pm_ops = {
        SET_RUNTIME_PM_OPS(ssd1681_runtime_suspend,
                           ssd1681_runtime_resume,
                           ssd1681_runtime_idle)
};
#else
static const struct dev_pm_ops ssd1681_pm_ops = {
        SET_RUNTIME_PM_OPS(NULL, NULL, NULL)
};
#endif

static struct spi_driver ssd1681_drv = {
        .probe    = ssd1681_probe,
        .remove   = ssd1681_remove,
        .id_table = ssd1681_spi_ids,
        .driver   = {
                .name           = DRV_NAME,
                .of_match_table = of_match_ptr(ssd1681_dt_ids),
                .pm             = &ssd1681_pm_ops
        },
};

module_spi_driver(ssd1681_drv);

MODULE_AUTHOR("Iota Hydrae <writeforever@foxmail.com>");
MODULE_DESCRIPTION("ssd1681 based 3/4-wire SPI LCD-TFT display framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:ssd1681");
MODULE_ALIAS("platform:ssd1681");
