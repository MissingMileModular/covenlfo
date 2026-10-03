# covenlfo
super witchy eurorack LFO Module based on SAMD21/ Xiao


![image of module](https://github.com/cctvfm/covenlfo/blob/main/IMG_E0173-01.jpeg)

This is the Missing Mile Modular fork of [cctvfm/covenlfo](https://github.com/cctvfm/covenlfo). It keeps the stock firmware untouched and adds custom firmware with five chaotic modes on top of the four stock waveforms. See [Missing Mile Modular Firmware](#missing-mile-modular-firmware) below.

# Missing Mile Modular Firmware

The hardware is unchanged: same pot, FM CV in, Sync CV in, button, and four outputs. Only the firmware differs.

## Sketches in this repo

Each folder is a self-contained Arduino sketch with its own copy of `antilog.h`. Each one builds on the one before it.

| Folder | Modes | What it adds |
|---|---|---|
| `LFOSAMD21.ino` (repo root) | 1–4 | Stock CCTV firmware, unmodified |
| `LFOSAMD21_dejong/` | 1–5 | De Jong attractor |
| `LFOSAMD21_dejong_w_SLEW/` | 1–6 | Slewed De Jong |
| `MMM_lfo_mk3/` | 1–7 | Sloth, LED intro animation |
| `MMM_lfo_mk4/` | 1–9 | Stooges, Cipher. **Current version** |

## Modes

A short press of the button steps to the next mode. After the last mode it wraps back to 1.

| # | Mode | What it does | Outputs 1–4 |
|---|---|---|---|
| 1 | Triangle | Stock | Divided-down copies |
| 2 | Saw | Stock | Divided-down copies |
| 3 | Square | Stock | Divided-down copies |
| 4 | Random | Stock stepped sample and hold | Divided-down copies |
| 5 | De Jong | Peter de Jong strange attractor: `x' = sin(a*y) - cos(b*x)`, `y' = sin(c*x) - cos(d*y)`. Pot and FM CV set the iteration rate (0.02–200 Hz) | x, y, previous x, previous y |
| 6 | De Jong Slew | Same attractor as mode 5, but the outputs glide to each new value instead of stepping | Same as mode 5 |
| 7 | Sloth | Emulation of the Nonlinear Circuits Sloth Chaos circuit. The pot sets the circuit's knob resistance, which biases which attractor the orbit favors. It does not set speed. FM CV feeds the circuit's CV input | Circuit nodes x, w, y, z |
| 8 | Stooges | Emulation of the Nonlinear Circuits Stooges / "Jerk Off": Sprott's jerk equation `x''' = -A*x'' - x' + abs(x) - 1` with A = 0.6 | x, x', x'', x''' |
| 9 | Cipher | 32-bit maximal-length LFSR in the style of the Nonlinear Circuits 8-Bit Cipher. Pot and FM CV set the clock rate | Four bit-windows read from the same register |

When the mode wraps from 9 back to 1, all four LEDs double-flash and then chase around the panel for about 1.2 seconds. The panel LEDs share the CV output pins, so this animation also appears as voltage at the jacks.

## Long press

A long press steps through three settings. What they select depends on the current mode.

| Mode | Setting 1 | Setting 2 | Setting 3 |
|---|---|---|---|
| 1–4 | Stock clock-division set | Stock clock-division set | Stock clock-division set |
| 5, 6 | Dense scribble | Wide fanned loops | Sparse web |
| 7 | Torpor, ~15–30 s orbit | Apathy, ~60–90 s orbit | Inertia, ~30–40 min orbit |
| 8 | Slow, ~0.1 Hz | Medium, ~2–3 Hz | Fast, near audio rate |
| 9 | Bit offsets 0, 7, 14, 21 | Bit offsets 0, 3, 11, 19 | Bit offsets 0, 5, 13, 23 |

Changing the setting in Sloth or Stooges resets the circuit state.

## How it works

The stock firmware computes its waveforms inside a 46 kHz timer interrupt. The chaotic modes need floating-point math that is too slow for that on a SAMD21, which has no FPU. So they are computed in `loop()` at their own rate and written to a shared buffer, and the interrupt only copies that buffer to the PWM outputs.

Design notes, known tuning points, and things still to verify on a scope are in [coven-lfo-notes.md](coven-lfo-notes.md).

## Building

Follow the Arduino setup in [How to Hack Your Coven LFO](#how-to-hack-your-coven-lfo) below, then open the `.ino` inside the folder you want, for example `MMM_lfo_mk4/MMM_lfo_mk4.ino`. The folder name must match the `.ino` name, and `antilog.h` must sit next to it.

There is no prebuilt UF2 for the custom firmware yet. `CovenLFO-9Bit.uf2` in the repo root is the stock CCTV firmware.

## Credits

- Original module and firmware: [CCTV](https://github.com/cctvfm/covenlfo), GPL-3.0
- Sloth Chaos and Stooges circuit designs: Andrew Fitch, Nonlinear Circuits
- Sloth math: ported from [Don Cross's reference implementation](https://github.com/cosinekitty/sloth)

# How to Hack Your Coven LFO  




If you are interested in modifying the code, you will first need to download the latest version of Arduino IDE which can be found here:  

https://www.arduino.cc/en/software 

Now, follow these instructions so that Arduino can recognize our board, the Seeeduino XIAO.  

https://wiki.seeedstudio.com/Seeeduino-XIAO/ 

Next, we need to install an additional library so that your code will compile. This library can be downloaded directly from inside the Arduino IDE by going to Tools> Manage Libraries... 

Once the Library Manager opens you can use the search bar located at the top to find the library we will need to install. Using the search bar, type in “FlashStorage_SAMD” and install the latest version. Note that this code was written with version 1.3.2 of the FlashStorage_SAMD library.

Select the Xiao Board: Tools -> Board -> Seeed SAMD -> Seeeduino Xiao

Once you have the Arduino IDE installed and recognizing your XIAO and have installed the “FlashStorage_SAMD” library, you are now ready to compile the code and modify it until your heart and ears are content.  

Read the comments for more info on what to HACK


# How to Update using UF2

Update can be done by dragging the UF2 file onto the boot disk. The boot disk appears by:

Connecting the Xiao to USB, **Do not connect Eurorack Power at the same time**

Double tap the reset pin pair with tweezers

Confirm that an "Arduino" Folder appears in Finder/Explorer.

Drag the UF2 file to the root disk directory.
