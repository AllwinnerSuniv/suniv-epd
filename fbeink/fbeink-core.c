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

#include "fbeink.h"

#define DRV_NAME "fbeink"

int fbeink_write_buf_dc(struct fbeink_par *par, void *buf, size_t len, int dc)
{
        int rc;

        gpiod_set_value(par->gpio.dc, dc);

        rc = fbeink_write_spi(par, buf, len);
        if (rc < 0)
                dev_err(&par->pdev->dev, "write() failed and returned %d\n", rc);

        return rc;
}
EXPORT_SYMBOL(fbeink_write_buf_dc);

int fbeink_probe_common(struct fbeink_display *display,
                    struct spi_device *sdev,
                    struct platform_device *pdev)
{
    struct device *dev;
    struct fb_info *info;
    struct fbeink_par *par;
    struct fbeink_platform_data *pdata;
    int ret;

    if (sdev)
        dev = &sdev->dev;
    else
        dev = &pdev->dev;

    // if (unlikely(display->debug))

    pdata = dev->platform_data;
    if (!pdata) {

    }

    return 0;
}
EXPORT_SYMBOL(fbeink_probe_common);

void fbeink_remove_common(struct device *dev, struct fb_info *info)
{

}
EXPORT_SYMBOL(fbeink_remove_common);

// MODULE_LICENSE("GPL");