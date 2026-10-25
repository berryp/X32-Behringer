#!/bin/sh
#
# Install the X32 Emulator virtual audio driver (run as root: sudo ./install.sh)
#
set -e
cd "$(dirname "$0")"
DRIVER=X32Emulator.driver
HALDIR=/Library/Audio/Plug-Ins/HAL

if [ ! -d "$DRIVER" ]; then
	echo "$DRIVER not found, build it first with: make"
	exit 1
fi
if [ "$(id -u)" -ne 0 ]; then
	echo "please run as root: sudo $0"
	exit 1
fi
mkdir -p "$HALDIR"
rm -rf "$HALDIR/$DRIVER"
cp -R "$DRIVER" "$HALDIR/"
chown -R root:wheel "$HALDIR/$DRIVER"
chmod -R 755 "$HALDIR/$DRIVER"
# reload CoreAudio so the new device appears
killall -9 coreaudiod 2>/dev/null || launchctl kickstart -kp system/com.apple.audio.coreaudiod 2>/dev/null || true
echo "Installed $HALDIR/$DRIVER - the \"X32 Emulator\" audio device is now available"
