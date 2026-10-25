#
# X32 Emulator virtual audio driver for macOS (CoreAudio AudioServerPlugIn)
#
#   make            build X32Emulator.driver (universal: arm64 + x86_64)
#   make install    install it in /Library/Audio/Plug-Ins/HAL and restart coreaudiod (sudo)
#   make uninstall  remove it and restart coreaudiod (sudo)
#   make test       build X32USBTest, a command line client to check the emulator's audio
#
# Needs the Xcode command line tools (xcode-select --install).
#
CC=clang
ARCHS=-arch arm64 -arch x86_64
CFLAGS=-O2 -Wall -Wextra -Wno-unused-parameter -mmacosx-version-min=10.15 $(ARCHS)
DRIVER=X32Emulator.driver
HALDIR=/Library/Audio/Plug-Ins/HAL
SRCS=X32USBDriver.c X32USBTransport.c
HDRS=X32USBTransport.h ../X32AudioProtocol.h

all: $(DRIVER)

$(DRIVER): $(SRCS) $(HDRS) Info.plist
	rm -rf $(DRIVER)
	mkdir -p $(DRIVER)/Contents/MacOS
	cp Info.plist $(DRIVER)/Contents/Info.plist
	$(CC) $(CFLAGS) -bundle -o $(DRIVER)/Contents/MacOS/X32Emulator $(SRCS) \
		-framework CoreAudio -framework CoreFoundation
	codesign --force --sign - $(DRIVER)

test: X32USBTest

X32USBTest: X32USBTest.c X32USBTransport.c $(HDRS)
	$(CC) -O2 -Wall -o $@ X32USBTest.c X32USBTransport.c -lpthread -lm

install: $(DRIVER)
	sudo ./install.sh

uninstall:
	sudo ./uninstall.sh

clean:
	rm -rf $(DRIVER) X32USBTest

.PHONY: all test install uninstall clean
