
# local kernel build dir
KERN_DIR ?= /lib/modules/$(shell uname -r)/build

# users kernel dir
KERN_DIR := /home/developer/sources/orangepi-build/kernel/orange-pi-5.10-rk3588

MODULE_NAME = ssd1327_fb

all:
	make -C $(KERN_DIR) M=`pwd` modules

clean:
	make -C $(KERN_DIR) M=`pwd` modules clean

clena: clean

#CFLAGS_$(MODULE_NAME).o := -DDEBUG
obj-m += $(MODULE_NAME).o
