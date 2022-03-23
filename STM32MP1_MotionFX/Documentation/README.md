In order to generate the MotionFX library, starting project is:

ssh://gitolite@codex.cro.st.com/nucleo-mems-shield-customer-fw/algorithms-libraries.git

Choose master branch

Fix motion_fx.[c,h] reverting slashes in linux style

Add the following Makefile to the path "algorithms-libraries/MotionFX/LibBuilder":

vpath %.o out

SRCS = interface/src/motion_fx.c

OBJS = *.o

CFLAGS += -Iinterface/inc -Icore/include -Icore/src -Icore/src/iNemo_Common_File -Icore/src/iNemo_Engine
CFLAGS += -Icore/mag_Cal_HI -Icore/src/mag_Cal_HI/include -Icore/src/mag_Cal_HI/src
CFLAGS += -Icore/src/mag_Cal_HI/src/interface -Icore/src/mag_Cal_HI/src/internal
CFLAGS += -Icore/src/mag_Cal_HI/src/internal/dataBuffer -Icore/src/mag_Cal_HI/src/internal/magCal -Icore/src/mag_Cal_HI/src/internal/utils
CFLAGS += -I.
LDFLAGS += -c -lm

OUT_DIR = out

default:
        mkdir -p $(OUT_DIR)
        $(CC) $(CFLAGS) $(LDFLAGS) $(INC) $(SRCS)               
        $(AR) rvs MotionFX_CM4F_GCC_os.a $(OBJS)
        mv *.o $(OUT_DIR)
        mkdir -p lib
        mv *.a lib
        cp interface/inc/motion_fx.h lib
        
clean:
        rm -rf $(OUT_DIR)
        rm -rf lib

Include also a cross compiler for gcc like build_pi4.sh:

PATH=$PATH:/local/home/tesim/RASPBERRY/tools/arm-bcm2708/arm-linux-gnueabihf/bin
make -j4 ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-

In documentation Folder also provided build and Makefile for RPI4 environment
