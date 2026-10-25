# X32 Emulator virtual audio driver for macOS

This driver replicates using an X32 as an audio interface on a Mac. With an
X-USB (or X-LIVE) expansion card, a real X32 shows up on macOS as a 32 in /
32 out USB audio device. Here, the X32 emulator (`X32`, in the parent
directory) plays the console, and this CoreAudio driver (an AudioServerPlugIn,
the same kind of driver as BlackHole or Loopback) plays the USB cable. It
publishes an audio device named **X32 Emulator** with 32 inputs and 32
outputs, and streams its audio to and from the emulator over UDP.

```
 macOS app (DAW, ...)                                  X32 emulator
 ┌────────────────────┐   UDP 127.0.0.1:10025   ┌──────────────────────────┐
 │ "X32 Emulator"     │ ── outputs 1-32 ──────▶ │ card inputs  CARD1-32    │──▶ /config/routing/IN|PLAY
 │ 32 in / 32 out     │ ◀── inputs 1-32 ─────── │ card outputs (32)        │◀── /config/routing/CARD
 └────────────────────┘                         └──────────────────────────┘
```

As on the real console:

- **Recording** (the device's inputs) gets the X32 *card outputs*: whatever
  `/config/routing/CARD/1-8 ... 25-32` sends to the card. These are the local
  inputs by default (`AN1-8` ... `AN25-32`). Other choices are `OUT1-8` and
  `OUT9-16` (the output patches `/outputs/main/01..16`: Main L/R, M/C, mix
  buses, channel direct outs) and `CARD1-8...` (loopback).
- **Playback** (the device's outputs) feeds the X32 *card inputs*. To hear or
  mix them, route input blocks to the card: `/config/routing/IN/1-8 ,i 16`
  (`CARD1-8`), or set the PLAY routing and `/config/routing/routswitch ,i 1`.
- **The console is the clock master**: the device runs at the emulator's
  `/-prefs/clockrate` (48 kHz or 44.1 kHz) and switches when it changes.
- **The card mode** `/-prefs/card/USBmode` (32/32, 16/16, 32/8, 8/32, 8/8, 2/2,
  read as X32→computer / computer→X32) limits how many channels carry audio.
  The device always shows 32 channels, and the unused ones are silent.
- `/meters/0`, `/meters/1` and `/meters/2` show live levels (input channels,
  mix buses, main LR, M/C), so X32-Edit and other remote apps show meters that
  move with the audio.

## What the emulator mixes

The emulator now has a small audio engine (`X32AudioEngine.c`). It implements
the signal path an audio interface setup needs, without any DSP effects:

- input routing (`/config/routing/IN`, `PLAY`, `routswitch`) and channel
  sources (`/ch/xx/config/source`, In01-32)
- channel trim, polarity, fader, mute, mute groups, DCAs, pan, LR assign and
  M/C level
- the 16 mix bus sends (pre/post-fader by tap type), and the mix bus masters with
  their fader, mute and LR/M/C assign
- Main LR (fader, mute, balance), M/C
- outputs 1-16 (`/outputs/main/NN/src`, `pos`) and the card output routing

It does not emulate EQ, dynamics, gates, insert/FX, aux inputs, AES50 (which is
silent), matrixes, P16 or monitor outputs. The emulator has no analog inputs,
so local inputs `AN1-32` are silent unless you start it with `-s 1` (input n
carries a sine at 100 Hz × n, -20 dBFS) or `-s 2` (white noise, -30 dBFS).

When no resource file exists yet (the first run), the emulator now starts from
audio-friendly defaults close to a real X32 factory state: channel n is fed by
input n, local inputs go to inputs 1-32 and to the card outputs, channels and
buses are on, outputs 1-2 are Main L/R. Faders start at -∞, as on the console.

## Build and install

You need the Xcode command line tools (`xcode-select --install`).

```sh
cd X32USBDriver
make                # builds X32Emulator.driver (universal arm64 + x86_64, ad-hoc signed)
sudo ./install.sh   # copies it to /Library/Audio/Plug-Ins/HAL and restarts coreaudiod
```

To remove it: `sudo ./uninstall.sh`.

Then build and run the emulator on the same Mac (from the repository root):

```sh
make X32
cd build && ./X32 -i 127.0.0.1 -s 1
```

Choose **X32 Emulator** as the input/output device in your DAW, or in Audio
MIDI Setup. With `-s 1` and the default routing, recording inputs 1-32 gives
the 32 test tones. The emulator logs when the driver connects and when IO
starts or stops.

### Example: play from the Mac through the console, record the mix

```
/config/routing/IN/1-8 ,i 16          inputs 1-8 come from CARD1-8 (Mac outputs 1-8)
/ch/01/mix/fader ,f 0.75              channel 1 at 0 dB
/main/st/mix/fader ,f 0.75            Main LR at 0 dB
/config/routing/CARD/9-16 ,i 20       card outputs 9-16 = OUT1-8 (OUT1/2 are Main L/R)
```

What the Mac plays on output 1 now comes back, mixed, on inputs 9 and 10.
Moving channel 1's fader or mute (from X32-Edit or X32_Command) changes what
you hear.

## Settings

The driver connects to `127.0.0.1:10025` by default. To use an emulator on
another machine or port, edit `X32EmulatorHost` and `X32EmulatorPort` in
`/Library/Audio/Plug-Ins/HAL/X32Emulator.driver/Contents/Info.plist` (use an
IP address), re-sign with `sudo codesign --force --sign - /Library/Audio/Plug-Ins/HAL/X32Emulator.driver`
and restart coreaudiod (`sudo killall coreaudiod`). Start the emulator with
`-a <port>` to change its audio port, or `-a 0` to turn its audio off.

Driver messages go to the system log:
`log stream --predicate 'eventMessage CONTAINS "X32USB"'`.

## Testing without installing the driver

`X32USBTest` runs the driver's network code (`X32USBTransport.c`) the way
CoreAudio does, from the command line. It works on macOS and Linux. It plays
1/2/3/4 kHz tones at -6 dBFS on outputs 1-4, records the 32 inputs, and
prints the level and frequency found on each one.

```sh
make test && ./X32USBTest -t 3
```

## Notes and limits

- Audio travels as 32-bit float over UDP. At 48 kHz that is about 6 MB/s in
  each direction, which is fine on the same Mac and on a wired LAN.
- Latency is a few milliseconds plus the jitter buffers: the driver reads input
  256 frames behind real time, and the emulator holds one IO buffer plus
  256 frames of playback. Round-trip latency is not fixed, so DAW latency
  compensation can be off by a few ms.
- The emulator and driver clocks both come from the Mac's host clock, so they
  do not drift on the same machine. Across machines the driver re-aligns when
  the drift gets too large, which causes a short dropout.
- The device's transport type is "virtual", and it has no volume controls (the
  X-USB card has none either).
- The device appears even when the emulator is not running. Its inputs are
  silent until the emulator answers.
