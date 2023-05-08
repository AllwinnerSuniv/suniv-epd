
# local kernel build dir
KERN_DIR ?= /lib/modules/$(shell uname -r)/build

# users kernel dir
KERN_DIR := /home/developer/sources/HamsterBear/software/bsp/linux-5.17.2

EINK_SRCS := fbeink-core.c fbeink-io.c
EINK_OBJS := $(subst .c,.o,$(EINK_SRCS))

all:
	make -C $(KERN_DIR) M=`pwd` modules

clean:
	make -C $(KERN_DIR) M=`pwd` modules clean

#CFLAGS_$(MODULE_NAME).o := -DDEBUG
obj-m += eink.o
eink-y += $(EINK_OBJS)

# obj-m += fbeink_uc8253.o
eink-y += fbeink_uc8253.o