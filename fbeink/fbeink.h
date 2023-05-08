#ifndef __LINUX_FBEINK_H
#define __LINUX_FBEINK_H

#include <linux/fb.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>

#define FBEINK_GPIO_NAME_SIZE    32

struct fbeink_gpio {
    char name[FBEINK_GPIO_NAME_SIZE];
    struct gpio_desc *gpio;
};

struct fbeink_par;

/**
 * struct fbeink_ops - FBEINK operations structure
 * @write: Writes to interface bus
 * @read: Reads from interface bus
 * @write_vmem: Writes video memory to display
 * @write_reg: Writes to controller register
 * @set_addr_win: Set the GRAM update window
 * @reset: Reset the LCD controller
 * @mkdirty: Marks display lines for update
 * @update_display: Updates the display
 * @init_display: Initializes the display
 * @blank: Blank the display (optional)
 * @request_gpios_match: Do pinname to gpio matching
 * @request_gpios: Request gpios from the kernel
 * @free_gpios: Free previously requested gpios
 * @verify_gpios: Verify that necessary gpios is present (optional)
 * @register_backlight: Used to register backlight device (optional)
 * @unregister_backlight: Unregister backlight device (optional)
 * @set_var: Configure LCD with values from variables like @rotate and @bgr
 *           (optional)
 * @set_gamma: Set Gamma curve (optional)
 *
 * Most of these operations have default functions assigned to them in
 *     fbeink_framebuffer_alloc()
 */
struct fbeink_ops {
        int (*write)(struct fbeink_par *par, void *buf, size_t len);
        int (*read)(struct fbeink_par *par, void *buf, size_t len);
        int (*write_vmem)(struct fbeink_par *par, size_t offset, size_t len);
        void (*write_register)(struct fbeink_par *par, int len, ...);
        
        void (*set_addr_win)(struct fbeink_par *par,
                             int xs, int ys, int xe, int ye);
        void (*reset)(struct fbeink_par *par);
        void (*mkdirty)(struct fb_info *info, int from, int to);
        void (*update_display)(struct fbeink_par *par,
                               unsigned int start_line, unsigned int end_line);
        int (*init_display)(struct fbeink_par *par);
        int (*blank)(struct fbeink_par *par, bool on);
        
        unsigned long (*request_gpios_match)(struct fbeink_par *par,
                                             const struct fbeink_gpio *gpio);
        int (*request_gpios)(struct fbeink_par *par);
        int (*verify_gpios)(struct fbeink_par *par);
        
        void (*register_backlight)(struct fbeink_par *par);
        void (*unregister_backlight)(struct fbeink_par *par);
        
        int (*set_var)(struct fbeink_par *par);
        int (*set_gamma)(struct fbeink_par *par, u32 *curves);
};

/**
 * struct fbeink_display - Describes the display properties
 * @width: Width of display in pixels
 * @height: Height of display in pixels
 * @regwidth: LCD Controller Register width in bits
 * @buswidth: Display interface bus width in bits
 * @backlight: Backlight type.
 * @fbeinkops: fbeink operations provided by driver or device (platform_data)
 * @bpp: Bits per pixel
 * @fps: Frames per second
 * @txbuflen: Size of transmit buffer
 * @init_sequence: Pointer to LCD initialization array
 * @gamma: String representation of Gamma curve(s)
 * @gamma_num: Number of Gamma curves
 * @gamma_len: Number of values per Gamma curve
 * @debug: Initial debug value
 *
 * This structure is not stored by fbeink except for init_sequence.
 */
struct fbeink_display {
        unsigned int width;
        unsigned int height;
        unsigned int regwidth;
        unsigned int buswidth;
        unsigned int backlight;
        struct fbeink_ops fbeinkops;
        unsigned int bpp;
        unsigned int fps;
        int txbuflen;
        const s16 *init_sequence;
        char *gamma;
        int gamma_num;
        int gamma_len;
        unsigned long debug;
};

/**
 * struct fbeink_platform_data - Passes display specific data to the driver
 * @display: Display properties
 * @gpios: Pointer to an array of pinname to gpio mappings
 * @rotate: Display rotation angle
 * @bgr: LCD Controller BGR bit
 * @fps: Frames per second (this will go away, use @fps in @fbeink_display)
 * @txbuflen: Size of transmit buffer
 * @startbyte: When set, enables use of Startbyte in transfers
 * @gamma: String representation of Gamma curve(s)
 * @extra: A way to pass extra info
 */
struct fbeink_platform_data {
        struct fbeink_display display;
        unsigned int rotate;
        bool bgr;
        unsigned int fps;
        int txbuflen;
        u8 startbyte;
        char *gamma;
        void *extra;
};

struct fbeink_par {
        struct spi_device *spi;
        struct platform_device *pdev;
        struct fb_info *info;
        struct fbeink_platform_data *pdata;
        u16 *ssbuf;
        u32 pseudo_palette[16];
        struct {
                void *buf;
                size_t len;
        } txbuf;
        u8 *buf;
        u8 startbyte;
        struct fbeink_ops fbeinkops;
        spinlock_t dirty_lock;
        unsigned int dirty_lines_start;
        unsigned int dirty_lines_end;
        struct {
                struct gpio_desc *reset;
                struct gpio_desc *dc;
                struct gpio_desc *rd;
                struct gpio_desc *wr;
                struct gpio_desc *latch;
                struct gpio_desc *cs;
                struct gpio_desc *db[16];
                struct gpio_desc *led[16];
                struct gpio_desc *aux[16];
        } gpio;
        const s16 *init_sequence;
        struct {
                struct mutex lock;
                u32 *curves;
                int num_values;
                int num_curves;
        } gamma;
        unsigned long debug;
        bool first_update_done;
        ktime_t update_time;
        bool bgr;
        void *extra;
        bool polarity;
};

#define NUMARGS(...)  (sizeof((int[]){__VA_ARGS__}) / sizeof(int))

#define write_reg(par, ...)                                            \
    ((par)->fbeinkops.write_register(par, NUMARGS(__VA_ARGS__), __VA_ARGS__))

/* fbeink-core.c */
int fbeink_write_buf_dc(struct fbeink_par *par, void *buf, size_t len, int dc);
__printf(5, 6)
void fbeink_dbg_hex(const struct device *dev, int groupsize,
           void *buf, size_t len, const char *fmt, ...);
struct fb_info *fbeink_framebuffer_alloc(struct fbeink_display *display,
                    struct device *dev,
                    struct fbeink_platform_data *pdata);
void fbeink_framebuffer_release(struct fb_info *info);
int fbeink_register_framebuffer(struct fb_info *fb_info);
int fbeink_unregister_framebuffer(struct fb_info *fb_info);
void fbeink_register_backlight(struct fbeink_par *par);
void fbeink_unregister_backlight(struct fbeink_par *par);
int fbeink_init_display(struct fbeink_par *par);
int fbeink_probe_common(struct fbeink_display *display, struct spi_device *sdev,
               struct platform_device *pdev);
void fbeink_remove_common(struct device *dev, struct fb_info *info);

/* fbeink-io.c */
int fbeink_write_spi(struct fbeink_par *par, void *buf, size_t len);
// int fbeink_write_spi_emulate_9(struct fbeink_par *par, void *buf, size_t len);
// int fbeink_read_spi(struct fbeink_par *par, void *buf, size_t len);
// int fbeink_write_gpio8_wr(struct fbeink_par *par, void *buf, size_t len);
int fbeink_write_gpio16_wr(struct fbeink_par *par, void *buf, size_t len);
int fbeink_write_gpio16_wr_latched(struct fbeink_par *par, void *buf, size_t len);

/* fbeink-bus.c */
// int fbeink_write_vmem8_bus8(struct fbeink_par *par, size_t offset, size_t len);
// int fbeink_write_vmem16_bus16(struct fbeink_par *par, size_t offset, size_t len);
// int fbeink_write_vmem16_bus8(struct fbeink_par *par, size_t offset, size_t len);
// int fbeink_write_vmem16_bus9(struct fbeink_par *par, size_t offset, size_t len);
// void fbeink_write_reg8_bus8(struct fbeink_par *par, int len, ...);
// void fbeink_write_reg8_bus9(struct fbeink_par *par, int len, ...);
// void fbeink_write_reg16_bus8(struct fbeink_par *par, int len, ...);
// void fbeink_write_reg16_bus16(struct fbeink_par *par, int len, ...);

#define FBEINK_REGISTER_DRIVER(_name, _compatible, _display)                \
									   \
static int fbeink_driver_probe_spi(struct spi_device *spi)                  \
{                                                                          \
	return fbeink_probe_common(_display, spi, NULL);                    \
}                                                                          \
									   \
static int fbeink_driver_remove_spi(struct spi_device *spi)                 \
{                                                                          \
	struct fb_info *info = spi_get_drvdata(spi);                       \
									   \
	fbeink_remove_common(&spi->dev, info);                              \
	return 0;                                                          \
}                                                                          \
									   \
static int fbeink_driver_probe_pdev(struct platform_device *pdev)           \
{                                                                          \
	return fbeink_probe_common(_display, NULL, pdev);                   \
}                                                                          \
									   \
static int fbeink_driver_remove_pdev(struct platform_device *pdev)          \
{                                                                          \
	struct fb_info *info = platform_get_drvdata(pdev);                 \
									   \
	fbeink_remove_common(&pdev->dev, info);                             \
	return 0;                                                          \
}                                                                          \
									   \
static const struct of_device_id dt_ids[] = {                              \
	{ .compatible = _compatible },                                     \
	{},                                                                \
};                                                                         \
									   \
MODULE_DEVICE_TABLE(of, dt_ids);                                           \
									   \
									   \
static struct spi_driver fbeink_driver_spi_driver = {                       \
	.driver = {                                                        \
		.name   = _name,                                           \
		.of_match_table = dt_ids,                                  \
	},                                                                 \
	.probe  = fbeink_driver_probe_spi,                                  \
	.remove = fbeink_driver_remove_spi,                                 \
};                                                                         \
									   \
static struct platform_driver fbeink_driver_platform_driver = {             \
	.driver = {                                                        \
		.name   = _name,                                           \
		.owner  = THIS_MODULE,                                     \
		.of_match_table = dt_ids,                                  \
	},                                                                 \
	.probe  = fbeink_driver_probe_pdev,                                 \
	.remove = fbeink_driver_remove_pdev,                                \
};                                                                         \
									   \
static int __init fbeink_driver_module_init(void)                           \
{                                                                          \
	int ret;                                                           \
									   \
	ret = spi_register_driver(&fbeink_driver_spi_driver);               \
	if (ret < 0)                                                       \
		return ret;                                                \
	ret = platform_driver_register(&fbeink_driver_platform_driver);     \
	if (ret < 0)                                                       \
		spi_unregister_driver(&fbeink_driver_spi_driver);           \
	return ret;                                                        \
}                                                                          \
									   \
static void __exit fbeink_driver_module_exit(void)                          \
{                                                                          \
	spi_unregister_driver(&fbeink_driver_spi_driver);                   \
	platform_driver_unregister(&fbeink_driver_platform_driver);         \
}                                                                          \
									   \
module_init(fbeink_driver_module_init);                                     \
module_exit(fbeink_driver_module_exit);

#define fbeink_REGISTER_SPI_DRIVER(_name, _comp_vend, _comp_dev, _display)	\
										\
static int fbeink_driver_probe_spi(struct spi_device *spi)			\
{										\
	return fbeink_probe_common(_display, spi, NULL);				\
}										\
										\
static int fbeink_driver_remove_spi(struct spi_device *spi)			\
{										\
	struct fb_info *info = spi_get_drvdata(spi);				\
										\
	fbeink_remove_common(&spi->dev, info);					\
	return 0;								\
}										\
										\
static const struct of_device_id dt_ids[] = {					\
	{ .compatible = _comp_vend "," _comp_dev },				\
	{},									\
};										\
										\
MODULE_DEVICE_TABLE(of, dt_ids);						\
										\
static const struct spi_device_id spi_ids[] = {					\
	{ .name = _comp_dev },							\
	{},									\
};										\
										\
MODULE_DEVICE_TABLE(spi, spi_ids);						\
										\
static struct spi_driver fbeink_driver_spi_driver = {				\
	.driver = {								\
		.name  = _name,							\
		.of_match_table = dt_ids,					\
	},									\
	.id_table = spi_ids,							\
	.probe  = fbeink_driver_probe_spi,					\
	.remove = fbeink_driver_remove_spi,					\
};										\
										\
module_spi_driver(fbeink_driver_spi_driver);

#endif