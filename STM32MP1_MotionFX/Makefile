
CFLAGS += -Wall -g -lm -D_GNU_SOURCE
CFLAGS += -I. -Ilinux/
LDFLAGS += -lm -lrt -lpthread -Llib/

.PHONY: clean

all: iio_test_sensors generic_buffer iio_event_monitor

timer_lib: lib/timer_lib.o
	$(CC) -c $@ $^ $(CFLAGS) $(LDFLAGS)

generic_buffer: generic_buffer.o iio_utils.o
	$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS)

iio_event_monitor: iio_event_monitor.o iio_utils.o
	$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS)

iio_test_sensors: iio_test_sensors.o iio_utils.o lib/timer_lib.o
	$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS) lib/MotionFX_CM4F_GCC_os.a

%.o: %.c iio_utils.h
	$(CC) -c $< -o $@ $(CFLAGS) $(LDFLAGS)

clean:
	rm -f *.o iio_event_monitor iio generic_buffer iio_test_sensors lib/timer_lib.o
