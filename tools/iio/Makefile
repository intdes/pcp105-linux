CC = $(CROSS_COMPILE)gcc
CFLAGS += -Wall -g -D_GNU_SOURCE -I/opt/OpenWrt/build_dir/target-i386_i486_musl-1.1.14/linux-x86_generic/linux-4.4.7/tools/include
LDFLAGS+= -L/opt/OpenWrt/build_dir/target-i386_i486_musl-1.1.14/linux-x86_generic/linux-4.4.7/tools/lib -lrt

all: iio_event_monitor lsiio generic_buffer sense_buffer

iio_event_monitor: iio_event_monitor.o iio_utils.o

lsiio: lsiio.o iio_utils.o

generic_buffer: generic_buffer.o iio_utils.o

sense_buffer: sense_buffer.o iio_utils.o

%.o: %.c iio_utils.h

.PHONY: clean
clean:
	rm -f *.o iio_event_monitor lsiio generic_buffer sense_buffer
