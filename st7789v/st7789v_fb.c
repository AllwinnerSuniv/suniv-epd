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
        int (*reset)(struct st7789v_par *par);
        int (*clear)(struct st7789v_par *par);
        int (*idle)(struct st7789v_par *par, bool on);
        int (*blank)(struct st7789v_par *par, bool on);
        int (*sleep)(struct st7789v_par *par, bool on);
        int (*set_addr_win)(struct st7789v_par *par, int xs, int ys, int xe, int ye);
};

struct st7789v_display {
        u32                     xres;
        u32                     yres;
        u32                     bpp;
        u32                     fps;
        u32                     rotate;
};

#define SUNIV_FIFO_DEPTH 128
struct st7789v_par {

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

        const struct st7789v_operations        *tftops;
        const struct st7789v_display           *display;

        struct tasklet_struct task;

        struct fb_info          *fbinfo;
        struct fb_ops           *fbops;

        u32             palette_buffer[256];
        u32             pseudo_palette[16];
};
u8 txbuf[SUNIV_FIFO_DEPTH];
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

#define NUMARGS(...)  (sizeof((int[]){__VA_ARGS__}) / sizeof(int))
static int st7789v_write_reg(struct st7789v_par *par, int len, ...)
{
        va_list args;
        u32 arg;
        int i;

        va_start(args, len);

        arg = va_arg(args, unsigned int);
        fbtft_write_buf_dc(par, &arg, 1, 0);
        len--;

        if (len == 0)
                return 0;

        for (i = 0; i < len; i++) {
                par->buf[i] = va_arg(args, unsigned int);
        }
        va_end(args);

        fbtft_write_buf_dc(par, par->buf, len, 1);

        return 0;
}
#define write_reg(par, ...) \
        st7789v_write_reg(par, NUMARGS(__VA_ARGS__), __VA_ARGS__)

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
        mdelay(50);

        write_reg(par, 0x11); //Sleep out
        mdelay(120);     //Delay 120ms
        //************* Start Initial Sequence **********//
        write_reg(par, 0x36, 0x00);

        write_reg(par, 0x3A, 0x05);

        write_reg(par, 0xB2, 0x0C, 0x0C, 0x00, 0x33, 0x33);

        write_reg(par, 0xB7, 0x35);

        write_reg(par, 0xBB, 0x32);     //Vcom=1.35V

        write_reg(par, 0xC2, 0x01);

        write_reg(par, 0xC3, 0x15);    //GVDD=4.8V

        write_reg(par, 0xC4, 0x20);     //VDV, 0x20:0v

        write_reg(par, 0xC6, 0x0F);   //0x0F:60Hz

        write_reg(par, 0xD0, 0xA4, 0xA1);

        write_reg(par, 0xE0, 0xD0, 0x08, 0x0E, 0x09, 0x09, 0x05, 0x31, 0x33, 0x48, 0x17, 0x14,
                  0x15, 0x31, 0x34);

        write_reg(par, 0xE1, 0xD0, 0x08, 0x0E, 0x09, 0x09, 0x15, 0x31, 0x33, 0x48, 0x17, 0x14,
                  0x15, 0x31, 0x34);

        write_reg(par, 0x21);

        write_reg(par, 0x29);
        return 0;
}

static int st7789v_blank(struct st7789v_par *par, bool on)
{
        if (on)
                write_reg(par, MIPI_DCS_SET_DISPLAY_OFF);
        else
                write_reg(par, MIPI_DCS_SET_DISPLAY_ON);
        return 0;
}

static int st7789v_set_addr_win(struct st7789v_par *par, int xs, int ys, int xe,
                                 int ye)
{
        dev_dbg(par->dev, "xs = %d, xe = %d, ys = %d, ye = %d\n", xs, xe, ys, ye);
        xs = xs + 20;
        xe = xe + 20;
        write_reg(par, MIPI_DCS_SET_COLUMN_ADDRESS,
                ((xs >> BITS_PER_BYTE) & 0xff), (xs & 0xff),
                ((xe >> BITS_PER_BYTE) & 0xff), (xe & 0xff));

        write_reg(par, MIPI_DCS_SET_PAGE_ADDRESS,
                ((ys >> BITS_PER_BYTE) & 0xff), (ys & 0xff),
                ((ye >> BITS_PER_BYTE) & 0xff), (ye & 0xff));

        write_reg(par, MIPI_DCS_WRITE_MEMORY_START);

        return 0;
}

static int st7789v_idle(struct st7789v_par *par, bool on)
{
        if (on)
                write_reg(par, MIPI_DCS_EXIT_IDLE_MODE);
        else
                write_reg(par, MIPI_DCS_EXIT_IDLE_MODE);

        return 0;
}

static int st7789v_sleep(struct st7789v_par *par, bool on)
{
        if (on) {
                write_reg(par, MIPI_DCS_SET_DISPLAY_OFF);
                write_reg(par, MIPI_DCS_ENTER_SLEEP_MODE);
        } else {
                write_reg(par, MIPI_DCS_EXIT_SLEEP_MODE);
                write_reg(par, MIPI_DCS_SET_DISPLAY_ON);
        }

        return 0;
}

static int st7789v_clear(struct st7789v_par *par)
{
        // int i;
        // st7789v_set_addr_win(par, 0, 0, 240, 320);
        // memset(par->txbuf, 0x78, 128);

        // for (i = 0; i < (240 * 280 * 2) / 128; i++) {
        //         fbtft_write_buf_dc(par, par->txbuf, 128, 1);
        // }
        return 0;
}

static const struct st7789v_operations default_st7789v_ops = {
        .idle  = st7789v_idle,
        .clear = st7789v_clear,
        .blank = st7789v_blank,
        .reset = st7789v_reset,
        .sleep = st7789v_sleep,
        .set_addr_win = st7789v_set_addr_win,
};

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

#define MADCTL_BGR BIT(3) /* bitmask for RGB/BGR order */
#define MADCTL_MV BIT(5) /* bitmask for page/column order */
#define MADCTL_MX BIT(6) /* bitmask for column address order */
#define MADCTL_MY BIT(7) /* bitmask for page address order */
static int st7789v_set_var(struct st7789v_par *par)
{
        u8 madctl_par = 0;

        switch (par->fbinfo->var.rotate) {
        case 0:
                break;
        case 90:
                madctl_par |= (MADCTL_MV | MADCTL_MY);
                break;
        case 180:
                madctl_par |= (MADCTL_MX | MADCTL_MY);
                break;
        case 270:
                madctl_par |= (MADCTL_MV | MADCTL_MX);
                break;
        default:
                return -EINVAL;

        }

        write_reg(par, MIPI_DCS_SET_ADDRESS_MODE, madctl_par);
        return 0;
}

static int st7789v_hw_init(struct st7789v_par *par)
{
        printk("%s, Display Panel initializing ...\n", __func__);
        st7789v_init_display(par);
        st7789v_set_var(par);
        st7789v_clear(par);

        return 0;
}

/* TODO: device seems received wrong color format, check data transfer routine */
static int write_vmem(struct st7789v_par *par, size_t offset, size_t len)
{
        u16 *vmem16;
        __be16 *txbuf16 = par->txbuf.buf;
        size_t remain, to_copy, tx_array_size;
        int i;

        dev_dbg(par->dev, "%s, offset = %d, len = %d\n", __func__, offset, len);

        remain = len / 2;
        vmem16 = (u16 *)(par->fbinfo->screen_buffer + offset);

        gpiod_set_value(par->gpio.dc, 1);

        /* non-buffered spi write */
        // if (!par->txbuf.buf)
                return fbtft_write_spi(par, vmem16, len);

        // tx_array_size = par->txbuf.len / 2;

        // while (remain) {
        //         to_copy = min(tx_array_size, remain);
        //         for (i = 0; i < to_copy; i++)
        //                 txbuf16[i] = cpu_to_be16(vmem16[i]);
        //         vmem16 = vmem16 + to_copy;

        //         /* send batch to device */
        //         fbtft_write_spi(par, txbuf16, to_copy);

        //         remain -= to_copy;
        // }
        return 0;
}

static void update_display(struct st7789v_par *par, unsigned int start_line,
                           unsigned int end_line)
{
        size_t offset, len;

        // printk("%s\n", __func__);
        dev_dbg(par->dev, "%s\n", __func__);

        par->tftops->idle(par, false);
        /* write vmem to display then call refresh routine */
        /*
         * when this was called, driver should wait for busy pin comes low
         * until next frame refreshed
         */
        start_line = 0;
        end_line = par->fbinfo->var.yres - 1;

        st7789v_set_addr_win(par, 0, start_line, par->fbinfo->var.xres - 1, end_line);

        offset = start_line * par->fbinfo->fix.line_length;
        len = (end_line - start_line + 1) * par->fbinfo->fix.line_length;

        write_vmem(par, offset, len);

        par->tftops->idle(par, true);
}

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

/* from pxafb.c */
static unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf) 
{
    chan &= 0xffff;
    chan >>= 16 - bf->length;
    return chan << bf->offset;
}

static int st7789v_fb_setcolreg(unsigned int regno, unsigned int red,
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

static const struct st7789v_display display = {
        .xres = 240,
        .yres = 280,
        .bpp = 16,
        .fps = 60,
        .rotate = 90,
};

static int st7789v_probe(struct spi_device *spi)
{
        struct device *dev = &spi->dev;
        struct st7789v_par *par;
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
        switch(rotate) {
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
        info = framebuffer_alloc(sizeof(struct st7789v_par), dev);
        if (!info) {
                dev_err(dev, "failed to alloc framebuffer!\n");
                return -ENOMEM;
        }

        info->screen_buffer = vmem;
        info->fbops = fbops;
        info->fbdefio = fbdefio;

        fbops->owner = dev->driver->owner;
        fbops->fb_read      = fb_sys_read;
        fbops->fb_write     = st7789v_fb_write;
        fbops->fb_blank     = st7789v_fb_blank;
        fbops->fb_fillrect  = st7789v_fb_fillrect;
        fbops->fb_copyarea  = st7789v_fb_copyarea;
        fbops->fb_imageblit = st7789v_fb_imageblit;
        fbops->fb_setcolreg = st7789v_fb_setcolreg;

        fbdefio->delay = HZ / display.fps;
        fbdefio->deferred_io = st7789v_deferred_io;
        fb_deferred_io_init(info);

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

        info->var.grayscale     = 0;

        info->flags = FBINFO_FLAG_DEFAULT | FBINFO_VIRTFB;

        /* st7789v self setup */
        par = info->par;
        info->pseudo_palette = &par->pseudo_palette;

        par->fbinfo = info;
        par->spi = spi;
        par->dev = dev;

        par->buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
        if (!par->buf) {
                dev_err(dev, "failed to alloc buf memory!\n");
                return -ENOMEM;
        }

        par->txbuf.buf = kmalloc(SUNIV_FIFO_DEPTH, GFP_KERNEL);
        if (!par->txbuf.buf) {
                dev_err(dev, "failed to alloc txbuf!\n");
                return -ENOMEM;
        }
        par->txbuf.len = SUNIV_FIFO_DEPTH;

        par->tftops = &default_st7789v_ops;
        par->display = &display;
        dev_set_drvdata(dev, par);
        spi_set_drvdata(spi, par);

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
        kfree(par->buf);
        kfree(par->txbuf.buf);
        unregister_framebuffer(par->fbinfo);
        framebuffer_release(par->fbinfo);
        return 0;
}

static int __maybe_unused st7789v_runtime_suspend(struct device *dev)
{
        struct st7789v_par *par = dev_get_drvdata(dev);

        par->tftops->sleep(par, true);

        return 0;
}

static int __maybe_unused st7789v_runtime_resume(struct device *dev)
{
        struct st7789v_par *par = dev_get_drvdata(dev);

        par->tftops->sleep(par, false);

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

#if CONFIG_PM
static const struct dev_pm_ops st7789v_pm_ops = {
        SET_RUNTIME_PM_OPS(NULL, NULL, NULL)
};
#else
static const struct dev_pm_ops st7789v_pm_ops = {
        SET_RUNTIME_PM_OPS(NULL, NULL, NULL)
};
#endif

static struct spi_driver st7789v_drv = {
        .probe    = st7789v_probe,
        .remove   = st7789v_remove,
        .id_table = st7789v_spi_ids,
        .driver   = {
                .name           = DRV_NAME,
                .of_match_table = of_match_ptr(st7789v_dt_ids),
                .pm             = &st7789v_pm_ops
        },
};

module_spi_driver(st7789v_drv);

MODULE_AUTHOR("Iota Hydrae <writeforever@foxmail.com>");
MODULE_DESCRIPTION("st7789v E-Paper display framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:st7789v");
