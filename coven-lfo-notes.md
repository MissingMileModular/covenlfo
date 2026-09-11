# Coven LFO — Custom Firmware Notes

Base hardware: [CCTV Coven LFO](https://github.com/cctvfm/covenlfo) — SAMD21 (Seeeduino
Xiao), 4HP, 4 bipolar CV outputs, 1 pot, FM CV in, Sync CV in, 1 button, 4 panel LEDs.
Stock firmware: 4 phase-accumulator waveforms (triangle/saw/square/random) with
divide-down sync. GPL-3.0, explicitly designed to be hacked.

Working file: `LFOSAMD21_dejong.ino` (+ unmodified `antilog.h` from the original repo).

## Why this needed real architectural changes, not just new waveforms

The stock firmware runs a 46kHz timer ISR (`TCC0_Handler`) that increments four phase
accumulators and calls `generator()` for simple periodic waveshapes — cheap enough to
run every PWM tick. Chaotic/attractor-style modes don't fit that model:

- They're feedback systems (next state depends on current state), not phase ramps.
- De Jong needs `sinf`/`cosf`; Sloth needs an iterative solver. Both are too slow to
  call from a 46kHz ISR on a Cortex-M0+ (no hardware FPU — all float math is
  software-emulated).

**Fix, used consistently across every chaotic mode**: do the expensive math in
`loop()` at a modest, self-timed rate (`micros()`-gated), write the result into a
shared output buffer (`jongOut[4]`), and have the ISR do nothing but copy that
buffer to the PWM compare registers. No float, no trig, no solver ever runs inside
the interrupt.

One more wrinkle worth remembering: **panel jack numbering doesn't match CC register
order.** The ISR writes `CC0→jack #4, CC1→jack #1, CC2→jack #2, CC3→jack #3` (see
comments in `TCC0_Handler`). Anything that needs to address a *specific physical
jack* (like the LED chase) has to go through that translation — see
`setPanelOutput()`.

## Waveform modes (`waveSelect`, cycled by short-press)

| # | Name | What it is |
|---|------|------------|
| 1 | TRIANGLE | stock |
| 2 | SAW | stock |
| 3 | SQUARE | stock |
| 4 | RANDOM | stock (stepped S&H) |
| 5 | DEJONG | Peter de Jong strange attractor: `x'=sin(ay)-cos(bx)`, `y'=sin(cx)-cos(dy)`. Iterated in `loop()` at 0.02–200Hz (rate derived from the same pot/CV curve as the other modes). Outputs: ch1/2 = current x/y, ch3/4 = *previous* iteration's x/y (one-step lag, cheap quadrature-ish pair). |
| 6 | DEJONG_SLEW | Same attractor as #5, but the ISR runs a fixed-point one-pole slew filter (`slewAccum[]`, `SLEWSHIFT`) toward each new value instead of stepping to it. Glides instead of jumps. |
| 7 | SLOTH | Numeric emulation of the NLC "Sloth Chaos" analog circuit (design: Andrew Fitch/NLC; math ported from [Don Cross's reference implementation](https://github.com/cosinekitty/sloth) for the official VCV port — full KCL derivation in that repo's README). Iterative convergence solver, no trig. POT sets knob resistance `K` (biases which strange attractor the orbit favors — *not* speed, faithful to real hardware). FM CV feeds the circuit's actual CV input `U`. Outputs: ch1–4 = circuit nodes x, w, y, z. |
| 8 | STOOGES | Numeric emulation of NLC's Stooges/"Jerk Off" — Sprott's simplest chaotic jerk equation `x'''=-A·x''-x'+|x|-1` (A=0.6). Confirmed as the circuit's actual basis by NLC designer Andrew Fitch on ModWiggler. Integrated with RK4, no trig, one `abs()`. Long-press cycles slow/medium/fast speed presets. Outputs: ch1–4 = x, x', x'', and the jerk term (x''') itself. |
| 9 | CIPHER | 8-Bit Cipher — not an ODE, a digital maximal-length 32-bit Galois LFSR (tap mask `0x80200003`, a commonly published maximal-length polynomial — kept as the only tap set used so correctness isn't a gamble). Clocked from the same pot/CV Hz curve as DEJONG's iteration rate (0.02–2000Hz). Long-press cycles which bit-window each of the 4 channels reads from the same register — channels are structurally correlated, unlike RANDOM's independent per-channel calls. Cheapest mode on the board: zero floats, zero trig. |

Wraps 9→1. **On wrap back to slot 1**, `ledIntroShow()` fires: double-flash all 4
LEDs, then chase 2→3→4→1 for two laps (~1.2s, blocking). Note: the 4 panel LEDs
share the CV output pins (no separate GPIO — confirmed by the firmware only ever
configuring pins 1/9/2/3 as outputs), so this animation is also a real voltage
excursion at whatever's patched into the jacks.

## Long-press (`divSelect`, 1..`DIVSIZE`=3) — meaning depends on current waveform

| waveSelect | What divSelect selects |
|---|---|
| 1–4 | `divs[]` — clock-division ratios (stock behavior) |
| 5, 6 | `jongPresets[]` — three `(a,b,c,d)` de Jong parameter sets (dense scribble / wide fanned loops / sparse web) |
| 7 | `slothPresets[]` — Torpor (~15-30s orbit) / Apathy (~60-90s) / Inertia (~30-40min); switching resets circuit state (fresh capacitor charge) |
| 8 | `stoogesPresets[]` — slow (~0.1Hz) / medium (~2-3Hz) / fast (near-audio) jerk-equation speed scale; switching resets state |
| 9 | `cipherOffsets[]` — three sets of bit-window read positions into the same LFSR (not different polynomials — see caveats below) |

Same physical DIVSIZE=3 array size and button gesture reused everywhere — no new UI
surface needed as modes were added.

## Known tuning points / things to revisit in code

- `SLEWSHIFT` (mode 6): fixed constant, not scaled to iteration rate. ~10-15ms time
  constant at 9. Could be made proportional to the de Jong iteration interval for a
  "glide time tracks rate knob" feel — not yet done.
- `scaleJong()` assumes de Jong x/y stay within ±2 (true for the shipped presets;
  a hand-tuned preset could diverge and get clamped).
- `scaleSloth()` assumes ±6V headroom on the Sloth circuit's x/w/y/z nodes — NLC's
  docs describe the hardware as mostly ±2V, occasionally ±4V, so there's margin, but
  it hasn't been scope-verified against this specific emulation's actual range.
- `SLOTH_QNEG`/`SLOTH_QPOS` (comparator saturation voltages) are Don Cross's
  breadboard-measured values, not datasheet values — a real first thing to check if
  Sloth's output range feels off once scoped.
- `scaleStooges()` assumes the A=0.6 jerk attractor stays within roughly ±3
  (dimensionless) units — not independently verified against a reference plot, just
  a starting assumption to check once you can scope it.
- `CIPHER`'s tap mask (`0x80200003`) is a commonly published maximal-length 32-bit
  Galois LFSR polynomial (matches Xilinx XAPP052 and others), but hasn't been
  independently verified in *this* implementation. Worth confirming the sequence
  doesn't degenerate to a short cycle before relying on it. The three long-press
  "presets" only change bit-window read offsets, not the polynomial — deliberately
  low-risk, since a wrong tap mask (not just a differently-sliced one) is what could
  actually break the sequence.
- De Jong "stretch goal" noted in-code: swap the ch3/ch4 lag-pair for a second fully
  independent attractor (different preset, different seed) — doubles sin/cos calls
  per iteration, still trivial at LFO rates.
- `ledIntroShow()` is blocking (~1.2s of `delay()`), consistent with the stock
  firmware's existing 2s blocking delay on long-press, but means Sync-input edges
  and pot reads are ignored during the animation.

## Build/flash

1. Arduino IDE, Seeeduino XIAO board package (`files.seeedstudio.com/arduino/package_seeeduino_boards_index.json`
   in Additional Board Manager URLs → Boards Manager → "Seeed SAMD").
2. Library Manager → install `FlashStorage_SAMD` (repo notes v1.3.2).
3. Sketch folder name must match the .ino filename. Put `antilog.h` (unmodified,
   from the original repo) in the same folder.
4. Tools → Board → Seeed SAMD → Seeeduino Xiao. Connect via USB (not Eurorack power
   at the same time). Double-tap the reset pads with tweezers if the port doesn't
   show up.
5. Verify, then Upload.

## File dependencies

- `LFOSAMD21_dejong.ino` — this file, everything described above.
- `antilog.h` — **unmodified**, copy from [cctvfm/covenlfo](https://github.com/cctvfm/covenlfo).
  Supplies the `hzcurve` pitch-lookup table used by stock frequency mapping (still
  used by modes 1–6 for their rate control).
