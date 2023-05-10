
# local kernel build dir
KERN_DIR ?= /lib/modules/$(shell uname -r)/build

# users kernel dir
KERN_DIR := /home/developer/sources/HamsterBear/software/bsp/linux-5.17.2

MODULE_NAME = eink_3in27

all:
	make -C $(KERN_DIR) M=`pwd` modules

clean:
	make -C $(KERN_DIR) M=`pwd` modules clean

clena: clean

CFLAGS_$(MODULE_NAME).o := -DDEBUG
obj-m += $(MODULE_NAME).o
