AR = $(CROSS_COMPILE)ar
CC := $(CROSS_COMPILE)gcc
LD := $(CROSS_COMPILE)gcc

vpath %.o out

SRCS = interface/src/motion_fx.c

OBJS = *.o

INC = -Iinterface/inc -Icore/include -Icore/src -Icore/src/iNemo_Common_File -Icore/src/iNemo_Engine
INC += -Icore/mag_Cal_HI -Icore/src/mag_Cal_HI/include -Icore/src/mag_Cal_HI/src
INC += -Icore/src/mag_Cal_HI/src/interface -Icore/src/mag_Cal_HI/src/internal
INC += -Icore/src/mag_Cal_HI/src/internal/dataBuffer -Icore/src/mag_Cal_HI/src/internal/magCal -Icore/src/mag_Cal_HI/src/internal/utils
INC += -I.

CFLAGS = -c -lm -Os -std=c99
#CFLAGS += -std=c99 -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
#CFLAGS += -frecord-gcc-switches -ffunction-sections -fdata-sections

OUT_DIR = out

default:
	mkdir -p $(OUT_DIR)
	$(CC) $(CFLAGS) $(INC) $(SRCS)		
	$(AR) rvs MotionFX_CM4F_GCC_os.a $(OBJS)
	mv *.o $(OUT_DIR)
	mkdir -p lib
	mv *.a lib
	cp interface/inc/motion_fx.h lib
	
clean:
	rm -rf $(OUT_DIR)
	rm -rf lib
