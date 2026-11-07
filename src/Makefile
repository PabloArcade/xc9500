# xc9500dump -- build with any c99 compiler and the C library only.
#   make            Linux (or any POSIX) binary: xc9500dump
#   make windows    Windows x86-64 binary with mingw-w64: xc9500dump.exe
# The Windows binary is linked with a 32 MB stack reserve: the decoder keeps
# several megabytes of per-device work arrays on the stack, and the default
# stack of a Windows thread is not enough. The Windows binary is stripped (the
# toolchain's runtime libraries carry debug sections otherwise) and carries no
# link time stamp, so that a rebuild gives the same bytes.
CC      = cc
CFLAGS  = -Wall -Wextra -O2 -std=c99
TARGET  = xc9500dump

SRCS = main.c device.c xc9500.c decode.c vhdl.c jed.c ucf.c target.c vendor_cells.c
OBJS = $(SRCS:.c=.o)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

main.o:   main.c jed.h device.h xc9500.h decode.h ucf.h vendor_cells.h
device.o: device.c device.h
xc9500.o: xc9500.c xc9500.h device.h jed.h decode.h
decode.o: decode.c decode.h xc9500.h device.h jed.h
vhdl.o:   vhdl.c decode.h xc9500.h device.h jed.h vendor_cells.h
jed.o:    jed.c jed.h
ucf.o:    ucf.c ucf.h
target.o: target.c target.h device.h jed.h
vendor_cells.o: vendor_cells.c vendor_cells.h vendor_cells_table.h device.h

WINCC     = x86_64-w64-mingw32-gcc
WINSTACK  = 33554432
WINTARGET = xc9500dump.exe

windows:
	$(WINCC) $(CFLAGS) -Wl,--strip-all -Wl,--no-insert-timestamp -Wl,--stack,$(WINSTACK) -o $(WINTARGET) $(SRCS)

clean:
	rm -f $(OBJS) $(TARGET) $(WINTARGET)

.PHONY: clean windows
