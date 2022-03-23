# !/bin/bash

PATH=$PATH:/local/home/tesim/RASPBERRY/tools/arm-bcm2708/arm-linux-gnueabihf/bin
make -j4 ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
