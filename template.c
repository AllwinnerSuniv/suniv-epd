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

#include <linux/uaccess.h>

#define DRV_NAME "template"

static int template_probe(struct platform_device *pdev)
{
    return 0;
}

static int template_remove(struct platform_device *pdev)
{
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id template_dt_ids[] = {
        { .compatible = "unknown,template" },
        { /* KEEP THIS */ },
};
#endif

static struct platform_driver template_drv = {
        .driver = {
                .name = DRV_NAME,
                .of_match_table = of_match_ptr(template_dt_ids),
        },
        .probe = template_probe,
        .remove = template_remove,
};

module_platform_driver(template_drv);

MODULE_AUTHOR("Iota Hydrae <writeforever@foxmail.com>");
MODULE_DESCRIPTION("linux platform driver template");
MODULE_LICENSE("GPL");

