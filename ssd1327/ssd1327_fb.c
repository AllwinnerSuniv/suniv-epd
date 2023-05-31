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

#include "imagedata.h"

/*
 * SSD1327 Command Table
 */
#define SSD1327_CMD_SET_COL_ADDR        0x15
#define SSD1327_CMD_SET_ROW_ADDR        0x75
#define SSD1327_CMD_SET_CONTRAST        0x81

/* 0x84 ~ 0x86 are no operation commands */

#define SSD1327_CMD_SET_REMAP                   0xA0
#define SSD1327_CMD_SET_DISPLAY_START_LINE      0xA1
#define SSD1327_CMD_SET_DISPLAY_OFFSET          0xA2

/* SSD1327 Display mode setting */
#define SSD1327_CMD_SET_DISPLAY_MODE_NORMAL          0xA4
#define SSD1327_CMD_SET_DISPLAY_MODE_ON              0xA5    /* All pixel at grayscale level GS15 */
#define SSD1327_CMD_SET_DISPLAY_MODE_OFF             0xA6    /* All pixel at grayscale level GS0 */
#define SSD1327_CMD_SET_DISPLAY_MODE_INVERSE         0xA7

#define SSD1327_CMD_SET_MULTIPLEX_RATIO         0xA8
#define SSD1327_CMD_FUNCTION_SELETION_A         0xAB

#define SSD1327_CMD_SET_DISPLAY_ON               0xAE
#define SSD1327_CMD_SET_DISPLAY_OFF              0xAF

#define SSD1327_CMD_SET_PHASE_LENGTH                0xB1
#define SSD1327_CMD_NOP                             0xB2
#define SSD1327_CMD_SET_FCLK_OSC_FREQ               0xB3
#define SSD1327_CMD_SET_GPIO                        0xB5
#define SSD1327_CMD_SET_SEC_PRE_CHARGE_PERIOD       0xB6
#define SSD1327_CMD_SET_GRAY_SCALE_TABLE            0xB8
#define SSd1327_CMD_SEL_DEF_LINEAR_GC_TABLE         0xB9

/* SSD1327 Default settings */
#define SSD1327_DEF_GAMMA_LEVEL         32

#define DRV_NAME "ssd1327_drv"

struct ssd1327_par;

struct ssd1327_operations {
        int (*init_display)(struct ssd1327_par *par);
        int (*reset)(struct ssd1327_par *par);
        int (*clear)(struct ssd1327_par *par);
        int (*idle)(struct ssd1327_par *par, bool on);
        int (*blank)(struct ssd1327_par *par, bool on);
        int (*sleep)(struct ssd1327_par *par, bool on);
        int (*set_var)(struct ssd1327_par *par);
        int (*set_gamma)(struct ssd1327_par *par, u8 gamma);
        int (*set_addr_win)(struct ssd1327_par *par, int xs, int ys, int xe, int ye);
};

struct ssd1327_display {
        u32                     xres;
        u32                     yres;
        u32                     bpp;
        u32                     fps;
        u32                     rotate;
        u32                     xs_off;
        u32                     xe_off;
        u32                     ys_off;
        u32                     ye_off;

        u8                      gamma;
};

#define SUNIV_FIFO_DEPTH 128
struct ssd1327_par {

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

        const struct ssd1327_operations        *tftops;
        const struct ssd1327_display           *display;

        struct fb_info          *fbinfo;
        struct fb_ops           *fbops;

        u32             pseudo_palette[16];

        u32             dirty_lines_start;
        u32             dirty_lines_end;
        u8              contrast;
};
// u8 txbuf[SUNIV_FIFO_DEPTH];
// static int g_epd_3in27_flag = 0;

static int fbtft_write_spi(struct ssd1327_par *par, void *buf, size_t len)
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

static int fbtft_write_buf_dc(struct ssd1327_par *par, void *buf, size_t len, int dc)
{
        int rc;

        gpiod_set_value(par->gpio.dc, dc);

        rc = fbtft_write_spi(par, buf, len);
        if (rc < 0)
                dev_err(par->dev, "write() failed and returned %d\n", rc);

        return rc;
}


static int fbtft_request_one_gpio(struct ssd1327_par *par,
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

                flags = (of_flags & OF_GPIO_ACTIVE_LOW) ? \
                        GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH;
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

static int fbtft_request_gpios(struct ssd1327_par *par)
{
        int rc;
        pr_debug("%s, configure from dt\n", __func__);

        rc = fbtft_request_one_gpio(par, "reset-gpios", 0, &par->gpio.reset);
        if (rc)
                return rc;
        rc = fbtft_request_one_gpio(par, "dc-gpios", 0, &par->gpio.dc);
        if (rc)
                return rc;
        rc = fbtft_request_one_gpio(par, "blk-gpios", 0, &par->gpio.blk);
        if (rc)
                return rc;
        rc = fbtft_request_one_gpio(par, "cs-gpios", 0, &par->gpio.cs);
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

static __inline int ssd1327_send(struct ssd1327_par *par, u8 byte, int dc)
{
        fbtft_write_buf_dc(par, &byte, 1, dc);
        return 0;
}
#define ssd1327_DC_DATA          1
#define ssd1327_DC_COMMAND       0
#define write_cmd(__par, __c) ssd1327_send(__par, __c, ssd1327_DC_COMMAND);
#define write_data(__par, __d) ssd1327_send(__par, __d, ssd1327_DC_DATA);

#define NUMARGS(...)  (sizeof((int[]){__VA_ARGS__}) / sizeof(int))
static int ssd1327_write_reg(struct ssd1327_par *par, int len, ...)
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
        ssd1327_write_reg(par, NUMARGS(__VA_ARGS__), __VA_ARGS__)

static int ssd1327_reset(struct ssd1327_par *par)
{
        gpiod_set_value_cansleep(par->gpio.reset, 1);
        msleep(10);
        gpiod_set_value_cansleep(par->gpio.reset, 0);
        msleep(100);
        gpiod_set_value_cansleep(par->gpio.reset, 1);
        msleep(10);
        return 0;
}

static int ssd1327_init_display(struct ssd1327_par *par)
{
        ssd1327_reset(par);

        write_reg(par, 0xae);//--turn off oled panel

        write_reg(par, 0x15);    //   set column address
        write_reg(par, 0x00);    //  start column   0
        write_reg(par, 0x7f);    //  end column   127

        write_reg(par, 0x75);    //   set row address
        write_reg(par, 0x00);    //  start row   0
        write_reg(par, 0x7f);    //  end row   127

        write_reg(par, 0x81);  // set contrast control
        write_reg(par, 0x80);

        write_reg(par, 0xa0);    // gment remap
        write_reg(par, 0x51);   //51

        write_reg(par, 0xa1);  // start line
        write_reg(par, 0x00);

        write_reg(par, 0xa2);  // display offset
        write_reg(par, 0x00);

        write_reg(par, 0xa4);    // rmal display
        write_reg(par, 0xa8);    // set multiplex ratio
        write_reg(par, 0x7f);

        write_reg(par, 0xb1);  // set phase leghth
        write_reg(par, 0xf1);

        write_reg(par, 0xb3);  // set dclk
        write_reg(par,
                  0x00);  //80Hz:0xc1 90Hz:0xe1   100Hz:0x00   110Hz:0x30 120Hz:0x50   130Hz:0x70     01

        write_reg(par, 0xab);  //
        write_reg(par, 0x01);  //

        write_reg(par, 0xb6);  // set phase leghth
        write_reg(par, 0x0f);

        write_reg(par, 0xbe);
        write_reg(par, 0x0f);

        write_reg(par, 0xbc);
        write_reg(par, 0x08);

        write_reg(par, 0xd5);
        write_reg(par, 0x62);

        write_reg(par, 0xfd);
        write_reg(par, 0x12);

        write_reg(par, 0xaf);

        return 0;
}

static int ssd1327_blank(struct ssd1327_par *par, bool on)
{
        if (on)
                write_reg(par, 0xa6);
        else
                write_reg(par, 0xa4);
        return 0;
}

static int ssd1327_set_addr_win(struct ssd1327_par *par, int xs, int ys, int xe,
                                int ye)
{
        /* set column adddress */
        write_reg(par, 0x15);

        /* 16grayscale cost 4bit which means one byte present 2 pixel */
        write_reg(par, xs / 2);
        write_reg(par, xe / 2);

        /* set row address */
        write_reg(par, 0x75);
        write_reg(par, ys);
        write_reg(par, ye);
        return 0;
}

static int ssd1327_idle(struct ssd1327_par *par, bool on)
{
        return 0;
}

static int ssd1327_sleep(struct ssd1327_par *par, bool on)
{
        if (on)
                write_reg(par, 0xaf);
        else
                write_reg(par, 0xae);

        printk("%s, panel was %s", __func__, on ? "off" : "on");
        return 0;
}

static int __maybe_unused ssd1327_show_img(struct ssd1327_par *par, const u8 *img)
{
        int i, j;
        int width = par->display->xres;
        int height = par->display->yres;
        int byte;
        ssd1327_set_addr_win(par, 0, 0, 127, 127);

        gpiod_set_value(par->gpio.dc, 1);
        for (i = 0; i < height; i++) {
                for (j = 0; j < width / 2; j++) {
                        /* 64 byte means one line length */
                        byte = img[j + i * 64];
                        fbtft_write_spi(par, &byte, 1);
                }
        }
        return 0;
}

static int __maybe_unused ssd1327_clear(struct ssd1327_par *par)
{
        int i;
        int width = par->display->xres;
        int height = par->display->yres;
        int clear = 0x00;
        ssd1327_set_addr_win(par, 0, 0, 127, 127);

        gpiod_set_value(par->gpio.dc, 1);
        for (i = 0; i < width * height * 16 / 8; i++)
                fbtft_write_spi(par, &clear, 1);
        return 0;
}

static int __maybe_unused ssd1327_set_var(struct ssd1327_par *par)
{
        return 0;
}

static int ssd1327_set_gamma(struct ssd1327_par *par, u8 gamma)
{
        /* The chip has 256 contrast steps from 0x00 to 0xFF */
        gamma &= 0xFF;

        write_reg(par, 0x81);
        write_reg(par, gamma);

        return 0;
}

static const struct ssd1327_operations default_ssd1327_ops = {
        .init_display = ssd1327_init_display,
        .idle         = ssd1327_idle,
        .clear        = ssd1327_clear,
        .blank        = ssd1327_blank,
        .reset        = ssd1327_reset,
        .sleep        = ssd1327_sleep,
        .set_var      = ssd1327_set_var,
        .set_gamma    = ssd1327_set_gamma,
        .set_addr_win = ssd1327_set_addr_win,
};

static int ssd1327_of_config(struct ssd1327_par *par)
{
        int rc;

        printk("%s\n", __func__);
        rc = fbtft_request_gpios(par);
        if (rc) {
                dev_err(par->dev, "Request gpios failed!\n");
                return rc;
        }
        return 0;

        /* request xres and yres from dt */
}

static int ssd1327_hw_init(struct ssd1327_par *par)
{
        u8 gamma = par->contrast = par->display->gamma;
        printk("%s, display panel initializing ...\n", __func__);

        par->tftops->init_display(par);
        par->tftops->set_var(par);
        par->tftops->set_gamma(par, (gamma & 0xFF));
        par->tftops->clear(par);
        // ssd1327_show_img(par, gImage_1in5);
        return 0;
}

#define RED(a)      ((((a) & 0xf800) >> 11) << 3)
#define GREEN(a)    ((((a) & 0x07e0) >> 5) << 2)
#define BLUE(a)     (((a) & 0x001f) << 3)
#define to_rgb565(r,g,b) ((r) << 11 | (g) << 5 | (b))
static inline u16 rgb565_to_grayscale_by_average(u16 rgb565)
{
        int r, g, b;
        u16 gray;

        r = RED(rgb565);
        g = GREEN(rgb565);
        b = BLUE(rgb565);

        gray = ((r + g + b) / 3);

        /* map to rgb565 format */
        r = b = gray * 31 / 255;  // 0 ~ 31
        g = gray * 63 / 255;

        return cpu_to_be16(to_rgb565(r, g, b));
}

static inline u16 rgb565_to_grayscale_by_weight(u16 rgb565)
{
        int r, g, b;
        u16 gray;

        /* get each channel and expand them to 8 bit */
        r = RED(rgb565);
        g = GREEN(rgb565);
        b = BLUE(rgb565);

        /* convert rgb888 to grayscale */
        gray = ((r * 77 + g * 151 + b * 28) >> 8); // 0 ~ 255

        /* map to rgb565 format */
        r = b = gray * 31 / 255;  // 0 ~ 31
        g = gray * 63 / 255;

        return cpu_to_be16(to_rgb565(r, g, b));
}

static inline u8 rgb565_to_4bit_grayscale(u16 rgb565)
{
        int r, g, b;
        __maybe_unused int level;
        u16 gray;

        /* get each channel and expand them to 8 bit */
        r = RED(rgb565);
        g = GREEN(rgb565);
        b = BLUE(rgb565);

        /* convert rgb888 to grayscale */
        gray = ((r * 77 + g * 151 + b * 28) >> 8); // 0 ~ 255
        if (gray == 0)
                return gray;

        /* to 4-bit grayscale */
        // gray = (gray * 15 / 255) & 0x0f;
        /*
         * so 4-bit grayscale like:
         * B3  B2  B1  B0
         * 0   0   0   0
         * which means have 16 kind of gray
         */
        gray /= 16;

        return gray;
}

static int write_vmem(struct ssd1327_par *par, size_t offset, size_t len)
{
        u16 *vmem16;
        // __be16 *txbuf16 = par->txbuf.buf;
        u8 *txbuf8 = par->txbuf.buf;
        size_t remain;
        size_t to_copy;
        size_t tx_array_size;
        __maybe_unused u8 p0, p1;  /* pixel 0,1 in this byte */
        int i, j;

        dev_dbg(par->dev, "%s, offset = %ld, len = %ld\n", __func__, offset, len);

        /*
         * rgb565 cost 2byte for one pixel,
         * so the remain of pixel count is
         * half of vmem_size
         */
        remain = len / 2;       /* remain pixel count */
        vmem16 = (u16 *)(par->fbinfo->screen_buffer + offset);

        gpiod_set_value(par->gpio.dc, 1);

        /* TODO:non-buffered spi write */
        if (!par->txbuf.buf) {
                dev_err(par->dev, "txbuf is NULL, this should never happen, do nothing...");
                return -ENOMEM;
                // return fbtft_write_spi(par, vmem16, len);
        }

        /*
         * some spi controller don't support sending too many bytes
         * at once, so we should send them in servral packets.
         */
        tx_array_size = par->txbuf.len;

        while (remain) {
                to_copy = min(tx_array_size, remain);
                dev_dbg(par->dev, "to_copy=%zu, remain=%zu\n",
                        to_copy, remain - to_copy);

                /*
                 * Well, the ssd1327 supports 16 depth grayscale display
                 * which means use 1 byte to control 2 pixels
                 * the memory in device like this:
                 *          SEG0         SEG1      ...   SEG 127
                 * COM0   Byte[3:0]    Byte[7:4]
                 * COM1
                 *  ...
                 * COM127
                 * pixel 0 stored in higher half of byte and
                 * pixel 1 stored in lower half of byte.
                 */
                for (i = 0, j = 0; i < to_copy; i += 2, j++) {

                        /* monochrome, don't use this for desktop env */
                        if (par->fbinfo->fix.visual == FB_VISUAL_MONO10) {
                                // txbuf16[i] = vmem16[i] ? 0 : 0xffff;
                                p0 = ((vmem16[i] ? 0xff : 0) & 0x0f);
                                p1 = ((vmem16[i + 1] ? 0xff : 0) & 0x0f);
                                txbuf8[j] = p0 << 4 | p1;
                        }

                        /* 16 depth grayscale */
                        if (par->fbinfo->var.grayscale) {
                                /* convert raw data to 4bit grayscale */
                                p0 = rgb565_to_4bit_grayscale(vmem16[i]);
                                p1 = rgb565_to_4bit_grayscale(vmem16[i + 1]);
                                txbuf8[j] = p0 << 4 | p1;
                        }
                }

                vmem16 = vmem16 + to_copy;

                /*
                 * send the batch to device
                 * we convert 2 raw rgb565 byte data in 1 byte
                 * each time in the loop up here, so the length
                 * of sending buffer should be halfed.
                 */
                fbtft_write_spi(par, par->txbuf.buf,  to_copy / 2);

                remain -= to_copy;
        }
        return 0;
}

static void update_display(struct ssd1327_par *par, unsigned int start_line,
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

        /* for each column, refresh dirty rows */
        par->tftops->set_addr_win(par, 0, start_line, par->fbinfo->var.xres - 1, end_line);

        offset = start_line * par->fbinfo->fix.line_length;
        len = (end_line - start_line + 1) * par->fbinfo->fix.line_length;

        write_vmem(par, offset, len);

        // par->tftops->idle(par, true);
}

static void ssd1327_mkdirty(struct fb_info *info, int y, int height)
{
        struct ssd1327_par *par = info->par;
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

static void ssd1327_deferred_io(struct fb_info *info, struct list_head *pagelist)
{
        struct ssd1327_par *par = info->par;
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

static void ssd1327_fb_fillrect(struct fb_info *info,
                                const struct fb_fillrect *rect)
{
        dev_dbg(info->dev,
                "%s: dx=%d, dy=%d, width=%d, height=%d\n",
                __func__, rect->dx, rect->dy, rect->width, rect->height);

        sys_fillrect(info, rect);
        ssd1327_mkdirty(info, rect->dy, rect->height);
}

static void ssd1327_fb_copyarea(struct fb_info *info,
                                const struct fb_copyarea *area)
{
        dev_dbg(info->dev,
                "%s: dx=%d, dy=%d, width=%d, height=%d\n",
                __func__,  area->dx, area->dy, area->width, area->height);

        sys_copyarea(info, area);
        ssd1327_mkdirty(info, area->dy, area->height);
}

static void ssd1327_fb_imageblit(struct fb_info *info,
                                 const struct fb_image *image)
{
        dev_dbg(info->dev,
                "%s: dx=%d, dy=%d, width=%d, height=%d\n",
                __func__,  image->dx, image->dy, image->width, image->height);
        sys_imageblit(info, image);

        ssd1327_mkdirty(info, image->dy, image->height);
}

static ssize_t ssd1327_fb_write(struct fb_info *info, const char __user *buf,
                                size_t count, loff_t *ppos)
{
        ssize_t res;
        dev_dbg(info->dev,
                "%s: count=%zd, ppos=%llu\n", __func__,  count, *ppos);

        res = fb_sys_write(info, buf, count, ppos);

        ssd1327_mkdirty(info, -1, 0);
        return 0;
}

/* from pxafb.c */
static unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
        chan &= 0xffff;
        chan >>= 16 - bf->length;
        return chan << bf->offset;
}

static int ssd1327_fb_setcolreg(unsigned int regno, unsigned int red,
                                unsigned int green, unsigned int blue,
                                unsigned int transp, struct fb_info *info)
{
        unsigned int val;
        int ret = 1;

        dev_dbg(info->dev, "%s(regno=%u, red=0x%X, green=0x%X, blue=0x%X, trans=0x%X)\n",
                __func__, regno, red, green, blue, transp);

        if (regno >= 256)   /* no. of hw registers */
                return 1;
        /*
        * Program hardware... do anything you want with transp
        */

        switch (info->fix.visual) {
        case FB_VISUAL_TRUECOLOR:
                if (regno < 16) {
                        val  = chan_to_field(red, &info->var.red);
                        val |= chan_to_field(green, &info->var.green);
                        val |= chan_to_field(blue, &info->var.blue);

                        ((u32 *)(info->pseudo_palette))[regno] = val;
                        ret = 0;
                }
                break;
        default:
                dev_err(info->dev, "unsupported color format!");
                ret = -1;
                break;
        }

        return ret;
}

static int ssd1327_fb_blank(int blank, struct fb_info *info)
{
        struct ssd1327_par *par = info->par;
        int ret = -EINVAL;

        switch (blank) {
        case FB_BLANK_POWERDOWN:
        case FB_BLANK_VSYNC_SUSPEND:
        case FB_BLANK_HSYNC_SUSPEND:
        case FB_BLANK_NORMAL:
                ret = ssd1327_blank(par, true);
                break;
        case FB_BLANK_UNBLANK:
                ret = ssd1327_blank(par, false);
                break;
        }
        return ret;
}

static ssize_t show_gamma_curve(struct device *device,
                                struct device_attribute *attr,
                                char *buf)
{
        struct ssd1327_par *par = dev_get_drvdata(device);

        return sprintf(buf, "%d\n", par->contrast);
}

static ssize_t store_gamma_curve(struct device *device,
                                 struct device_attribute *attr,
                                 const char *buf, size_t count)
{
        struct ssd1327_par *par = dev_get_drvdata(device);
        u8 contrast;
        int rc;

        rc = kstrtou8(buf, 0, &contrast);
        if (rc) {
                dev_err(device, "unsupported gamma value");
                return -EINVAL;
        }

        par->contrast = contrast;
        par->tftops->set_gamma(par, contrast);

        return count;
}

static struct device_attribute gamma_device_attrs[] = {
        __ATTR(gamma, 0660, show_gamma_curve, store_gamma_curve),
};

static const struct ssd1327_display display = {
        .xres = 128,
        .yres = 128,
        .bpp = 16,
        .fps = 60,
        .rotate = 0,

        .gamma = SSD1327_DEF_GAMMA_LEVEL,
};

static int ssd1327_probe(struct spi_device *spi)
{
        struct device *dev = &spi->dev;
        struct ssd1327_par *par;
        struct fb_deferred_io *fbdefio;
        int width, height, bpp, rotate;
        struct fb_info *info;
        struct fb_ops *fbops;
        u8 *vmem = NULL;
        int vmem_size;
        __maybe_unused int rc;

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
        info = framebuffer_alloc(sizeof(struct ssd1327_par), dev);
        if (!info) {
                dev_err(dev, "failed to alloc framebuffer!\n");
                return -ENOMEM;
        }

        info->screen_buffer = vmem;
        info->fbops = fbops;
        info->fbdefio = fbdefio;

        fbops->owner        = dev->driver->owner;
        fbops->fb_read      = fb_sys_read;
        fbops->fb_write     = ssd1327_fb_write;
        fbops->fb_fillrect  = ssd1327_fb_fillrect;
        fbops->fb_copyarea  = ssd1327_fb_copyarea;
        fbops->fb_imageblit = ssd1327_fb_imageblit;
        fbops->fb_setcolreg = ssd1327_fb_setcolreg;
        fbops->fb_blank     = ssd1327_fb_blank;

        snprintf(info->fix.id, sizeof(info->fix.id), "%s", dev->driver->name);
        info->fix.type            =       FB_TYPE_PACKED_PIXELS;
        info->fix.visual          =       FB_VISUAL_TRUECOLOR;
        // info->fix.visual          =       FB_VISUAL_MONO10;
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
        info->var.grayscale       =       1;

        switch (info->var.bits_per_pixel) {
        case 1:
        case 2:
        case 4:
        case 8:
                info->var.red.offset = info->var.green.offset = info->var.blue.offset = 0;
                info->var.red.length = info->var.green.length = info->var.blue.length = 8;
                break;

        case 16:
                info->var.red.offset      =       11;
                info->var.red.length      =       5;
                info->var.green.offset    =       5;
                info->var.green.length    =       6;
                info->var.blue.offset     =       0;
                info->var.blue.length     =       5;
                info->var.transp.offset   =       0;
                info->var.transp.length   =       0;
                break;
        default:
                dev_err(dev, "color depth %d not supported\n",
                        info->var.bits_per_pixel);
                break;
        }

        info->flags = FBINFO_FLAG_DEFAULT | FBINFO_VIRTFB;

        fbdefio->delay = HZ / display.fps;
        fbdefio->deferred_io = ssd1327_deferred_io;
        fb_deferred_io_init(info);

        /* ssd1327 self setup */
        par = info->par;
        info->pseudo_palette = &par->pseudo_palette;

        par->fbinfo = info;
        par->spi = spi;
        par->dev = dev;

        par->buf = devm_kzalloc(dev, PAGE_SIZE, GFP_KERNEL);
        if (!par->buf) {
                dev_err(dev, "failed to alloc buf memory!\n");
                return -ENOMEM;
        }

        par->txbuf.buf = devm_kzalloc(dev, PAGE_SIZE, GFP_KERNEL);
        if (!par->txbuf.buf) {
                dev_err(dev, "failed to alloc txbuf!\n");
                return -ENOMEM;
        }
        par->txbuf.len = PAGE_SIZE;

        par->tftops = &default_ssd1327_ops;
        par->display = &display;

        dev_set_drvdata(dev, par);
        spi_set_drvdata(spi, par);

        spin_lock_init(&par->dirty_lock);
        init_completion(&par->complete);
        ssd1327_of_config(par);
        ssd1327_hw_init(par);

        update_display(par, 0, par->fbinfo->var.yres - 1);

        /* framebuffer register */
        rc = register_framebuffer(info);
        if (rc < 0) {
                dev_err(dev, "framebuffer register failed with %d!\n", rc);
                goto alloc_fail;
        }

        /* create sysfs interface */
        device_create_file(dev, &gamma_device_attrs[0]);

        printk("%zu KB buffer memory\n", par->txbuf.len >> 10);
        printk(" spi%d.%d at %d MHz\n", spi->master->bus_num, spi->chip_select,
               spi->max_speed_hz / 1000000);
        printk("%d KB video memory\n", info->fix.smem_len >> 10);

        return 0;

alloc_fail:
        vfree(vmem);

        return 0;
}

static int ssd1327_remove(struct spi_device *spi)
{
        struct ssd1327_par *par = spi_get_drvdata(spi);

        printk("%s\n", __func__);
        fb_deferred_io_cleanup(par->fbinfo);

        unregister_framebuffer(par->fbinfo);
        framebuffer_release(par->fbinfo);

        device_remove_file(par->fbinfo->dev, &gamma_device_attrs[0]);
        return 0;
}

static int __maybe_unused ssd1327_runtime_suspend(struct device *dev)
{
        struct ssd1327_par *par = dev_get_drvdata(dev);

        par->tftops->sleep(par, true);

        return 0;
}

static int __maybe_unused ssd1327_runtime_resume(struct device *dev)
{
        struct ssd1327_par *par = dev_get_drvdata(dev);

        par->tftops->sleep(par, false);

        return 0;
}

static int __maybe_unused ssd1327_runtime_idle(struct device *dev)
{
        struct ssd1327_par *par = dev_get_drvdata(dev);

        par->tftops->idle(par, true);

        return 0;
}

static const struct of_device_id ssd1327_dt_ids[] = {
        { .compatible = "ultrachip,uc8253" },
        { /* KEEP THIS */ },
};
MODULE_DEVICE_TABLE(of, ssd1327_dt_ids);

static const struct spi_device_id ssd1327_spi_ids[] = {
        { "uc8253" },
        { /* KEEP THIS */ },
};
MODULE_DEVICE_TABLE(spi, ssd1327_spi_ids);

#if CONFIG_PM
static const struct dev_pm_ops ssd1327_pm_ops = {
        SET_RUNTIME_PM_OPS(ssd1327_runtime_suspend,
                           ssd1327_runtime_resume,
                           ssd1327_runtime_idle)
};
#else
static const struct dev_pm_ops ssd1327_pm_ops = {
        SET_RUNTIME_PM_OPS(NULL, NULL, NULL)
};
#endif

static struct spi_driver ssd1327_drv = {
        .probe    = ssd1327_probe,
        .remove   = ssd1327_remove,
        .id_table = ssd1327_spi_ids,
        .driver   = {
                .name           = DRV_NAME,
                .of_match_table = of_match_ptr(ssd1327_dt_ids),
                .pm             = &ssd1327_pm_ops
        },
};

module_spi_driver(ssd1327_drv);

MODULE_AUTHOR("Iota Hydrae <writeforever@foxmail.com>");
MODULE_DESCRIPTION("ssd1327 based 3/4-wire SPI LCD-TFT display framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:ssd1327");
MODULE_ALIAS("platform:ssd1327");
