#!/bin/bash

if [ X$1 = X ]; then
	sudo dmesg | tail -10
else
	sudo dmesg | tail -$1
fi
