#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/module.h>

#include <video/mipi_display.h>

#include "fbeink.h"

#define DRVNAME "fbeink_uc8253"

static irqreturn_t panel_busy_handler(int irq, void *data)
{
        return IRQ_HANDLED;
}

static int init_display(struct fbeink_par *par)
{
        par->fbeinkops.reset(par);
        mdelay(50);

        return 0;
}

static struct fbeink_display display = {
        .regwidth = 8,
        .width = 360,
        .height = 240,
        .fbeinkops = {
                .init_display = init_display,
        },
};

FBEINK_REGISTER_DRIVER(DRVNAME, "ultrachip,uc8253", &display);

MODULE_ALIAS("spi:" DRVNAME);
MODULE_ALIAS("platform:" DRVNAME);
MODULE_ALIAS("spi:uc8253");
MODULE_ALIAS("platform:uc8253");

MODULE_DESCRIPTION("FB driver for the UC8253 E-ink controller");
MODULE_AUTHOR("Iota Hydrae");
MODULE_LICENSE("GPL");