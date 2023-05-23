#!/bin/bash

echo "building..."
make

echo "reinstalling mod ..."
sudo rmmod ssd1306_fb
sudo insmod ssd1306_fb.ko

echo "dumping log..."
dmesg | tail -50
