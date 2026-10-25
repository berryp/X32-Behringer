#!/bin/sh
#
# Remove the X32 Emulator virtual audio driver (run as root: sudo ./uninstall.sh)
#
set -e
DRIVER=/Library/Audio/Plug-Ins/HAL/X32Emulator.driver

if [ "$(id -u)" -ne 0 ]; then
	echo "please run as root: sudo $0"
	exit 1
fi
rm -rf "$DRIVER"
killall -9 coreaudiod 2>/dev/null || launchctl kickstart -kp system/com.apple.audio.coreaudiod 2>/dev/null || true
echo "Removed $DRIVER"
