#!/bin/bash

echo "building..."
make

if [ "$?" != "0" ]; then
    echo "build failed! exiting..."
    exit -1
fi

echo "reinstalling mod ..."
sudo rmmod ssd1306_fb
sudo insmod ssd1306_fb.ko

echo "dumping log..."
dmesg | tail -50
