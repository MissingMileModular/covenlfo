//V1.1 + De Jong attractor mode
//Base: cctvfm/covenlfo March 30 2023 (ryokell out-of-order divs / non-1 first div)
//Added: DEJONG waveform (Peter de Jong strange attractor as a 4th/5th "waveshape")
//
// DESIGN NOTES (read this before you tweak it further):
//
// The de Jong system is:
//    x' = sin(a*y) - cos(b*x)
//    y' = sin(c*x) - cos(d*y)
// It's a feedback map, not a periodic waveform, so it can't live inside the
// existing phase-accumulator/generator() scheme. It also needs sinf/cosf,
// which are too slow to call from the 46kHz TCC0 ISR on this M0+ (no FPU).
//
// So: the attractor is iterated in loop() at a rate derived from the existing
// pot/CV frequency curve (sweepValue), clamped to 0.02-200 Hz. Each iteration
// writes the four output values into jongOut[4]. TCC0_Handler() just copies
// jongOut[] straight to the PWM compare registers when waveSelect==DEJONG -
// no floating point in the ISR, ever.
//
// Outputs: ch1/ch2 = current x/y. ch3/ch4 = the *previous* iteration's x/y,
// i.e. a one-sample lag. That gives you a usable quadrature-ish companion
// pair for free without doubling the compute cost. If you want two fully
// independent attractors instead (richer, but 2x the sin/cos calls per
// iteration), see the comment at the bottom of the DEJONG block in loop().
//
// Long-press (the button action that normally cycles divide-ratios) cycles
// jongPresets[] instead, while waveSelect==DEJONG - same UI gesture,
// different meaning depending on mode. This reuses divSelect/DIVSIZE as-is.

#include <FlashAsEEPROM_SAMD.h>
#include <avr/pgmspace.h>
#include <math.h>
#include "antilog.h"
#include <Arduino.h>

#define TRIANGLE 1
#define SAW      2
#define SQUARE   3
#define RANDOM   4
#define DEJONG      5   // NEW
#define DEJONG_SLEW 6   // NEW: same attractor, slewed/glided output
#define SLOTH       7   // NEW: NLC Sloth Chaos circuit emulation
#define STOOGES     8   // NEW: NLC Stooges/"Jerk Off" - Sprott jerk chaos
#define CIPHER      9   // NEW: NLC 8-Bit Cipher - maximal-length LFSR

#define FREQ 0
#define POT  1

//#define HZPHASOR 91183 //phasor value for 1 hz.
#define HZPHASOR 91625

long unsigned int accumulator1 = 0;
long unsigned int accumulator2 = 0;
long unsigned int accumulator3 = 0;
long unsigned int accumulator4 = 0;
long unsigned int phasor1;
long unsigned int phasor2;
long unsigned int phasor3;
long unsigned int phasor4;
unsigned int randNum[4];

FlashStorage(div_storage, int);
FlashStorage(wave_storage, int);
FlashStorage(init_storage, char);

////////////////////////////////////////////////////////////////////////////////////
// DIVIDE DOWN ARRAYS //
// Add more if you want! //
////////////////////////////////////////////////////////////////////////////////////
#define DIVSIZE 3 // if you add more divide down arrays, increase this number from 3 (the default number of arrays) to how many arrays you have total

char divs[DIVSIZE][4] = {{1,3,7,11},
                          {1,2,4,8},
                          {1,4,8,16}}; // You could add more divide down arrays here

////////////////////////////////////////////////////////////////////////////////////
// DE JONG ATTRACTOR PARAMETER PRESETS //
// Selected via the same long-press action that picks a divs[] row above, //
// but only while waveSelect == DEJONG. Keep DIVSIZE rows here too. //
////////////////////////////////////////////////////////////////////////////////////
float jongPresets[DIVSIZE][4] = {
  { 1.40f, -2.30f,  2.40f, -2.10f},  // classic - dense scribble
  {-2.70f, -0.09f, -0.86f, -2.20f},  // wide fanned loops
  { 2.01f, -2.53f,  1.61f, -0.33f}   // looser, sparser web
};

float jongX = 0.1f, jongY = 0.1f;   // attractor state
volatile unsigned int jongOut[4] = {255, 255, 255, 255}; // written in loop(), read in ISR
unsigned long lastJongUpdate = 0;   // micros() timestamp of last iteration

////////////////////////////////////////////////////////////////////////////////////
// SLOTH: numeric emulation of the Nonlinear Circuits "Sloth Chaos" analog circuit //
// (design: Andrew Fitch/NLC). Math and component values ported from Don Cross's //
// reference implementation for the official VCV port: //
// https://github.com/cosinekitty/sloth //
// Same derivation (KCL on each op-amp node, iterated to convergence each step), //
// re-expressed in plain C/float with no std:: dependencies for the M0+. //
////////////////////////////////////////////////////////////////////////////////////
#define SLOTH_C1 2.0e-6f
#define SLOTH_C2 1.42e-6f   // schematic says 1uF; measured "slothier" at this value
#define SLOTH_C3 50.0e-6f
#define SLOTH_R1 1.0e+6f
#define SLOTH_R2 4.7e+6f
#define SLOTH_R4 100.0e+3f
#define SLOTH_R5 100.0e+3f
#define SLOTH_R6 100.0e+3f
#define SLOTH_R7 100.0e+3f
#define SLOTH_R8 470.0e+3f
#define SLOTH_QNEG -10.64f  // op-amp comparator saturation voltages,
#define SLOTH_QPOS  11.38f  // measured from real hardware by Don Cross

// Torpor / Apathy / Inertia - orbit-speed presets, {timeDilation, w0 (initial
// charge on C3)}. Selected the same way as the De Jong presets: long-press
// cycles divSelect while waveSelect == SLOTH.
float slothPresets[DIVSIZE][2] = {
  {1.0f,          0.0f},    // Torpor  - ~15-30s orbit
  {0.27391343f,   0.017f},  // Apathy  - ~60-90s orbit
  {0.009697118f, -0.023f}   // Inertia - ~30-40min orbit
};

float slothX = 0.0f, slothW = 0.0f, slothY = 0.0f, slothZ = 0.0f;
int slothActivePreset = -1;         // -1 forces an init on first entry to SLOTH mode
unsigned long lastSlothUpdate = 0;  // micros() timestamp of last iteration

float slothQ(float z)
{
  // The comparator U1 output responds immediately - saturated inverting amp.
  return (z < 0.0f) ? SLOTH_QPOS : SLOTH_QNEG;
}

void slothInit(int presetIdx)
{
  slothW = slothPresets[presetIdx][1];
  slothX = 0.0f;
  slothY = 0.0f;
  slothZ = 0.0f;
  slothActivePreset = presetIdx;
}

// One convergence-solver step, advancing the circuit by dtSeconds of simulated
// time. K = total knob resistance in ohms (R3 fixed + R9 variable),
// U = control voltage in volts fed in through R8. presetIdx picks the
// timeDilation factor (Torpor/Apathy/Inertia).
void slothUpdate(float dtSeconds, float K, float U, int presetIdx)
{
  float dt = slothPresets[presetIdx][0] * dtSeconds;

  float xm = slothX, wm = slothW, zm = slothZ;
  float Qm = slothQ(zm);
  float ex = 0.0f, ew = 0.0f, ey = 0.0f;
  const float tol2 = 1.0e-9f; // solver tolerance, squared (float precision)

  float x2 = slothX, w2 = slothW, y2 = slothY, z2 = slothZ;

  for (int iter = 1; iter <= 5; iter++) {
    float dx = -dt / SLOTH_C1 * (zm / SLOTH_R1 + Qm / SLOTH_R2 + wm / K);
    float dw =  dt / SLOTH_C3 * (xm / SLOTH_R6 - (1.0f / SLOTH_R6 + 1.0f / K + 1.0f / SLOTH_R7) * wm);
    float dy = (-dt / (SLOTH_R7 * SLOTH_C2)) * wm;

    x2 = slothX + dx;
    w2 = slothW + dw;
    y2 = slothY + dy;
    // z responds instantaneously - no capacitor in this op-amp's feedback loop.
    z2 = -SLOTH_R4 * (y2 / SLOTH_R5 + U / SLOTH_R8);

    if (iter > 1) {
      float ddx = dx - ex, ddw = dw - ew, ddy = dy - ey;
      float variance = ddx * ddx + ddw * ddw + ddy * ddy;
      if (variance < tol2 || iter >= 5) break;
    }

    // Mean-value estimates over the time step, for the next iteration.
    xm = slothX + dx / 2.0f;
    wm = slothW + dw / 2.0f;
    zm = (slothZ + z2) / 2.0f;

    if (slothZ * z2 >= 0.0f) {
      Qm = slothQ(zm);
    }
    else {
      // z crossed zero mid-step - weight Q by how far into the step it happened.
      float alpha = slothZ / (slothZ - z2);
      Qm = alpha * slothQ(slothZ) + (1.0f - alpha) * slothQ(z2);
    }

    ex = dx; ew = dw; ey = dy;
  }

  slothX = x2;
  slothW = w2;
  slothY = y2;
  slothZ = z2;
}

// Hardware Sloth reportedly wanders +/-2V mostly, +/-4V occasionally - give it
// +/-6V of headroom before clamping to the 0-511 PWM range.
unsigned int scaleSloth(float v)
{
  float s = (v + 6.0f) * (511.0f / 12.0f);
  if (s < 0.0f) s = 0.0f;
  if (s > 511.0f) s = 511.0f;
  return (unsigned int)s;
}

////////////////////////////////////////////////////////////////////////////////////
// STOOGES: Sprott's simplest chaotic jerk equation - x''' = -A*x'' - x' + |x| - 1. //
// Confirmed as the actual basis for NLC's Stooges/"Jerk Off" circuit by the //
// designer (Andrew Fitch, on ModWiggler): "based on Sprott's chaotic jerk //
// equation and is pretty much a straight conversion from the equation to a //
// circuit." A=0.6 is Sprott's own robustly-chaotic value. //
// //
// Cheapest chaos mode on the board - one abs(), no trig, no iterative solver. //
// Integrated with RK4 for stability margin at the faster presets. Outputs are //
// literally position/velocity/acceleration/jerk (x, x', x'', x'''). //
////////////////////////////////////////////////////////////////////////////////////
#define STOOGES_A 0.6f

// Speed presets: natural-time-units of simulated time per real second.
// Selected via long-press, same pattern as every other mode's presets.
float stoogesPresets[DIVSIZE] = {
  1.0f,   // slow   - roughly a tenth-Hz chaotic wobble
  4.0f,   // medium - close to the ~2-3Hz the real Jerk Off self-oscillates at
  10.0f   // fast   - near-audio bounce
};

float stoogesX = 0.5f, stoogesY = 0.0f, stoogesZ = 0.0f;
int stoogesActivePreset = -1;
unsigned long lastStoogesUpdate = 0;

void stoogesInit(int presetIdx)
{
  stoogesX = 0.5f; // origin isn't an equilibrium here, but starting slightly
  stoogesY = 0.0f;  // off it gets the chaos moving immediately either way
  stoogesZ = 0.0f;
  stoogesActivePreset = presetIdx;
}

// RK4 step of the jerk system, time-scaled by the selected preset.
void stoogesUpdate(float dtSeconds, int presetIdx)
{
  float dt = stoogesPresets[presetIdx] * dtSeconds;
  float x = stoogesX, y = stoogesY, z = stoogesZ;

  float k1x = y;
  float k1y = z;
  float k1z = -STOOGES_A * z - y + fabsf(x) - 1.0f;

  float k2x = y + dt / 2.0f * k1y;
  float k2y = z + dt / 2.0f * k1z;
  float k2z = -STOOGES_A * (z + dt / 2.0f * k1z) - (y + dt / 2.0f * k1y) + fabsf(x + dt / 2.0f * k1x) - 1.0f;

  float k3x = y + dt / 2.0f * k2y;
  float k3y = z + dt / 2.0f * k2z;
  float k3z = -STOOGES_A * (z + dt / 2.0f * k2z) - (y + dt / 2.0f * k2y) + fabsf(x + dt / 2.0f * k2x) - 1.0f;

  float k4x = y + dt * k3y;
  float k4y = z + dt * k3z;
  float k4z = -STOOGES_A * (z + dt * k3z) - (y + dt * k3y) + fabsf(x + dt * k3x) - 1.0f;

  stoogesX = x + dt / 6.0f * (k1x + 2.0f * k2x + 2.0f * k3x + k4x);
  stoogesY = y + dt / 6.0f * (k1y + 2.0f * k2y + 2.0f * k3y + k4y);
  stoogesZ = z + dt / 6.0f * (k1z + 2.0f * k2z + 2.0f * k3z + k4z);
}

// This particular jerk attractor is dimensionless (not real volts) and
// empirically stays within roughly +/-3 units for A=0.6 - not independently
// verified against a reference plot, so treat as a starting assumption to
// check once you can scope it.
unsigned int scaleStooges(float v)
{
  float s = (v + 3.0f) * (511.0f / 6.0f);
  if (s < 0.0f) s = 0.0f;
  if (s > 511.0f) s = 511.0f;
  return (unsigned int)s;
}

////////////////////////////////////////////////////////////////////////////////////
// CIPHER: 8-Bit Cipher - not an ODE, a digital maximal-length LFSR, matching //
// the original's CMOS shift-register/XOR-tap character. Clocked from the //
// same pot/CV Hz curve every other rate-controlled mode uses - here it's a //
// literal clock rate rather than a simulated-time scale, so it maps directly. //
// //
// Tap mask 0x80200003 (bits 32,22,2,1) is a commonly published maximal-length //
// 32-bit Galois LFSR polynomial (Xilinx XAPP052 and others cite the same //
// value) - kept as the only tap set so correctness isn't a gamble. The three //
// long-press "presets" instead vary which bit-window each channel reads from //
// the same register, which can't break the sequence, only how it's sliced. //
////////////////////////////////////////////////////////////////////////////////////
int cipherOffsets[DIVSIZE][4] = {
  {0, 7, 14, 21},
  {0, 3, 11, 19},
  {0, 5, 13, 23}
};

unsigned long cipherLFSR = 0xACE1UL; // any nonzero seed works
unsigned long lastCipherUpdate = 0;

unsigned long cipherStep(unsigned long lfsr)
{
  unsigned long lsb = lfsr & 1UL;
  lfsr >>= 1;
  if (lsb) lfsr ^= 0x80200003UL;
  return lfsr;
}

////////////////////////////////////////////////////////////////////////////////////
// DEJONG_SLEW: fixed-point one-pole slew toward jongOut[], run every ISR tick. //
// Accumulator is kept as value<<16 so the glide keeps creeping even when the //
// remaining distance is under 1 LSB of the 9-bit output - a plain integer //
// >>SHIFT filter would stall short of the target instead of arriving. //
////////////////////////////////////////////////////////////////////////////////////
#define SLEWSHIFT 9   // higher = slower/creamier glide, lower = snappier
long slewAccum[4] = {0, 0, 0, 0}; // fixed point, real value = slewAccum>>16

////////////////////////////////////////////////////////////////////////////////////

char debounceState = 0;
unsigned long int debounceTime = 0;
int waveSelect = 1;
int divSelect = 1;
unsigned long lastSettingsSave = 0;
bool Mode = 0; // 0 = POT and 1 = SYNC
float sweepValue;
long unsigned int Time1 = 0;
long unsigned int Time2 = 0;
long unsigned int Periud = 0; // Period (arduino didn't allow use of word "period")
float syncFrequency = 0;

void timerIsr();
void setupTimers();
void TCC0_Handler();
void ledIntroShow(); // NEW
void slothInit(int presetIdx);                                    // NEW
void slothUpdate(float dtSeconds, float K, float U, int presetIdx); // NEW
unsigned int scaleSloth(float v);                                 // NEW
void stoogesInit(int presetIdx);                                  // NEW
void stoogesUpdate(float dtSeconds, int presetIdx);                // NEW
unsigned int scaleStooges(float v);                               // NEW
unsigned long cipherStep(unsigned long lfsr);                     // NEW

// +++++++++++++++++++++++++++++++++++ SETUP ++++++++++++++++++++++++++++++++++++++++
void setup() {
  pinMode(1, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(13, OUTPUT);       // using pin 13 to check interupt on timer 1
  pinMode(6, INPUT_PULLUP);  // pin 7 pushbutton to select waveform
  pinMode(A10, INPUT);       // used for analogReading sync (cv2)

  readSettings();
  //Serial.begin(9600);
  setupTimers(); // **this may not be the right location
  randomSeed(analogRead(A8));
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++ MAIN LOOP +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void loop() {
  static int modeCounter = 0;
  bool lastButtonState = 0;
  static bool buttonState = 0;
  static int syncState = 0; // used to find the leading edge to calculate period (static so it isn't updated to zero each loop)
  static int syncCounter = 0;
  int Sync;

  ////////////////////////////////////////////////////////////////////////////////////
  // "EEPROM" Save //
  // Store the settings after 60 seconds //
  ////////////////////////////////////////////////////////////////////////////////////
  if ((millis() - lastSettingsSave) > 60000) {
    saveSettings();
    lastSettingsSave = millis();
  }

  ////////////////////////////////////////////////////////////////////////////////////
  // Sync mode //
  // sync the 1:1 output with an incoming signal //
  ////////////////////////////////////////////////////////////////////////////////////
  Sync = analogRead(A10); // CV2 pin A10 on XIAO

  if ((Sync < 370) && (syncState == 0)) { // RISING EDGE
    Time1 = micros();
    syncState = 1;
  }
  else if ((Sync > 370) && (syncState == 1)) { // FALLING
    syncState = 2;
  }
  else if ((Sync < 370) && (syncState == 2)) { // NEXT RISING EDGE
    syncState = 3;
    Time2 = micros();
    Mode = 1;
    Periud = (Time2 - Time1);
    syncFrequency = 1000000;
    syncFrequency = syncFrequency / Periud;

    if (divs[divSelect - 1][0] == 1) accumulator1 = 0;
    if (divs[divSelect - 1][1] == 1) accumulator2 = 0;
    if (divs[divSelect - 1][2] == 1) accumulator3 = 0;
    if (divs[divSelect - 1][3] == 1) accumulator4 = 0;

    Time1 = Time2;

    if (syncCounter == 0) {
      accumulator1 = 0;
      accumulator2 = 0;
      accumulator3 = 0;
      accumulator4 = 0;
    }
    if (syncCounter % divs[divSelect - 1][0] == 0) accumulator1 = 0;
    if (syncCounter % divs[divSelect - 1][1] == 0) accumulator2 = 0;
    if (syncCounter % divs[divSelect - 1][2] == 0) accumulator3 = 0;
    if (syncCounter % divs[divSelect - 1][3] == 0) accumulator4 = 0;

    if ((syncCounter % divs[divSelect - 1][0] == 0) &&
        (syncCounter % divs[divSelect - 1][1] == 0) &&
        (syncCounter % divs[divSelect - 1][2] == 0) &&
        (syncCounter % divs[divSelect - 1][3] == 0)) {
      syncCounter = 0;
    }
    syncCounter++;
  }
  else if ((Sync > 370) && (syncState == 3)) { // last falling edge
    syncState = 2;
  }
  ////////////////////////////////////////////////////////////
  // END OF SYNC STUFF //
  ////////////////////////////////////////////////////////////

  if (digitalRead(6) == LOW && debounceState == 0) {
    debounceState = 1;
    debounceTime = millis();
  }
  else if (debounceState == 1) {
    if ((millis() - debounceTime) > 80) {
      if (digitalRead(6) == LOW) debounceState = 2;
      else debounceState = 0;
    }
  }
  else if (debounceState == 2) {
    if (digitalRead(6) == HIGH) {
      debounceState = 0;
      if ((millis() - debounceTime) < 3000) { // short press: change waveform
        waveSelect++;
        if (waveSelect > 9) { // NEW: was >4, now includes DEJONG..CIPHER
          waveSelect = 1;
          ledIntroShow(); // NEW: wrapped back to slot 1 - double flash + chase
        }
      }
    }
    else if ((millis() - debounceTime) > 3000) { // long press: change divs / jong preset
      debounceState = 3;
      debounceTime = millis();
      accumulator1 = 0;
      accumulator2 = 0;
      accumulator3 = 0;
      accumulator4 = 0;
      phasor1 = 0;
      phasor2 = 0;
      phasor3 = 0;
      phasor4 = 0;
      delay(2000);
      divSelect++;
      if (divSelect > DIVSIZE) {
        divSelect = 1;
      }
    }
  }
  else if (debounceState == 3) { // holding only
    if ((millis() - debounceTime) > 3000) {
      debounceState = 3;
      debounceTime = millis();
      divSelect++;
      if (divSelect > DIVSIZE) {
        divSelect = 1;
      }
    }
    else if (digitalRead(6) == HIGH) {
      debounceState = 0;
    }
  }

  float tempphasor;
  int cv1Value;
  static int potValue;
  static int oldpotValue;

  filterPut(POT, analogRead(A0));
  potValue = filterGet(POT);

  modeCounter++;
  if (modeCounter > 100) {
    if ((potValue - oldpotValue) > 20 || (oldpotValue - potValue) > 20) {
      Mode = 0;
    }
    oldpotValue = potValue;
    modeCounter = 0;
  }

  filterPut(FREQ, analogRead(A4));
  cv1Value = filterGet(FREQ);
  cv1Value = 1023 - cv1Value;
  cv1Value = cv1Value - 565;

  if (Mode == 1) {
    sweepValue = syncFrequency;
  }
  else {
    if ((Mode == 0) && ((cv1Value > 20) || (cv1Value < -20))) {
      int totalcv = potValue + cv1Value;
      if (totalcv < 0) totalcv = 0;
      else if (totalcv > 1023) totalcv = 1023;
      sweepValue = pgm_read_float_near(hzcurve + totalcv);
    }
    else {
      sweepValue = pgm_read_float_near(hzcurve + potValue);
    }
  }

  tempphasor = sweepValue * HZPHASOR;
  phasor1 = (unsigned long int)tempphasor / divs[divSelect - 1][0];
  phasor2 = (unsigned long int)tempphasor / divs[divSelect - 1][1];
  phasor3 = (unsigned long int)tempphasor / divs[divSelect - 1][2];
  phasor4 = (unsigned long int)tempphasor / divs[divSelect - 1][3];

  ////////////////////////////////////////////////////////////////////////////////////
  // De Jong attractor iteration (runs for both DEJONG and DEJONG_SLEW - they share //
  // the same underlying attractor; DEJONG_SLEW just glides to each new value //
  // instead of stepping to it, handled down in TCC0_Handler). //
  // Iterates at sweepValue Hz (same pot/CV curve everything else uses), //
  // clamped so sinf/cosf never has to run faster than the loop can keep up. //
  ////////////////////////////////////////////////////////////////////////////////////
  if (waveSelect == DEJONG || waveSelect == DEJONG_SLEW) {
    unsigned long nowUs = micros();
    float iterHz = sweepValue;
    if (iterHz < 0.02f) iterHz = 0.02f;
    if (iterHz > 200.0f) iterHz = 200.0f;
    unsigned long jongIntervalUs = (unsigned long)(1000000.0f / iterHz);

    if ((nowUs - lastJongUpdate) >= jongIntervalUs) {
      lastJongUpdate = nowUs;

      float *p = jongPresets[divSelect - 1];
      float newX = sinf(p[0] * jongY) - cosf(p[1] * jongX);
      float newY = sinf(p[2] * jongX) - cosf(p[3] * jongY);

      // ch3/ch4 get the outgoing (pre-update) x/y - a one-iteration lag,
      // cheap quadrature-ish companion to ch1/ch2.
      jongOut[2] = scaleJong(jongX);
      jongOut[3] = scaleJong(jongY);

      jongX = newX;
      jongY = newY;

      jongOut[0] = scaleJong(jongX);
      jongOut[1] = scaleJong(jongY);

      // STRETCH GOAL: for two fully independent attractors instead of a
      // lagged pair, keep a second (jongX2, jongY2) state seeded slightly
      // differently and iterate it here with jongPresets[(divSelect) % DIVSIZE]
      // (the *next* preset), writing its scaled output to jongOut[2]/[3]
      // instead of the lag values above. Doubles the sin/cos calls per
      // iteration but that's still trivial at LFO rates.
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////
  // Sloth chaos circuit emulation (only runs while waveSelect == SLOTH). //
  // POT sets the knob resistance K exactly as on real hardware - it doesn't set //
  // a "rate", it biases which strange attractor the orbit favors. FM CV input //
  // feeds the circuit's own CV input U, same as patching into the real module. //
  ////////////////////////////////////////////////////////////////////////////////////
  if (waveSelect == SLOTH) {
    int presetIdx = divSelect - 1;
    if (presetIdx != slothActivePreset) {
      slothInit(presetIdx); // first entry to SLOTH mode, or preset just changed
    }

    unsigned long nowUs = micros();
    if ((nowUs - lastSlothUpdate) >= 5000UL) { // ~200Hz internal update rate -
                                                // plenty fine for time constants
                                                // measured in tenths of a second
      float dtSeconds = (nowUs - lastSlothUpdate) / 1000000.0f;
      lastSlothUpdate = nowUs;

      float knobFraction = potValue / 1023.0f;             // 0..1 from the pot
      float K = 100.0e+3f + (knobFraction * 10.0e+3f);     // R3 (100k) + R9 (10k pot)
      float U = cv1Value * (12.0f / 512.0f);                // FM CV -> circuit CV input
      if (U > 12.0f) U = 12.0f;
      if (U < -12.0f) U = -12.0f;

      slothUpdate(dtSeconds, K, U, presetIdx);

      // Reuses the same jongOut[] buffer the ISR already reads for DEJONG -
      // see TCC0_Handler. No slew needed here, the circuit's own RC filtering
      // already makes these outputs naturally smooth.
      jongOut[0] = scaleSloth(slothX);
      jongOut[1] = scaleSloth(slothW);
      jongOut[2] = scaleSloth(slothY);
      jongOut[3] = scaleSloth(slothZ);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////
  // Stooges (jerk chaos) - only runs while waveSelect == STOOGES. //
  ////////////////////////////////////////////////////////////////////////////////////
  if (waveSelect == STOOGES) {
    int presetIdx = divSelect - 1;
    if (presetIdx != stoogesActivePreset) {
      stoogesInit(presetIdx);
    }

    unsigned long nowUs = micros();
    if ((nowUs - lastStoogesUpdate) >= 1000UL) { // 1kHz internal update - keeps
                                                  // the RK4 step small relative
                                                  // to the natural timescale
                                                  // even at the fast preset
      float dtSeconds = (nowUs - lastStoogesUpdate) / 1000000.0f;
      lastStoogesUpdate = nowUs;

      stoogesUpdate(dtSeconds, presetIdx);

      float jerk = -STOOGES_A * stoogesZ - stoogesY + fabsf(stoogesX) - 1.0f;

      jongOut[0] = scaleStooges(stoogesX);
      jongOut[1] = scaleStooges(stoogesY);
      jongOut[2] = scaleStooges(stoogesZ);
      jongOut[3] = scaleStooges(jerk);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////
  // 8-Bit Cipher - only runs while waveSelect == CIPHER. Clocked from the same //
  // Hz curve as DEJONG's iteration rate, used here as a literal clock rate. //
  ////////////////////////////////////////////////////////////////////////////////////
  if (waveSelect == CIPHER) {
    unsigned long nowUs = micros();
    float clockHz = sweepValue;
    if (clockHz < 0.02f) clockHz = 0.02f;
    if (clockHz > 2000.0f) clockHz = 2000.0f;
    unsigned long cipherIntervalUs = (unsigned long)(1000000.0f / clockHz);

    if ((nowUs - lastCipherUpdate) >= cipherIntervalUs) {
      lastCipherUpdate = nowUs;
      cipherLFSR = cipherStep(cipherLFSR);

      int *off = cipherOffsets[divSelect - 1];
      jongOut[0] = (unsigned int)((cipherLFSR >> off[0]) & 0x1FFUL);
      jongOut[1] = (unsigned int)((cipherLFSR >> off[1]) & 0x1FFUL);
      jongOut[2] = (unsigned int)((cipherLFSR >> off[2]) & 0x1FFUL);
      jongOut[3] = (unsigned int)((cipherLFSR >> off[3]) & 0x1FFUL);
    }
  }
}

// +++++++++++++++++++++++++++++++++++++++++ FUNCTION DEFINITIONS ++++++++++++++++++++++++++++++++++

unsigned long int previous_acc[4];

unsigned int generator(unsigned long int acc, char waveshape, char channel) {
  //December 12 - Adjusted for 9 bit
  unsigned int shifted_acc = acc >> 23;
  #define HALFPOINT 255
  #define MAXPOINT 511

  if (waveshape == SQUARE) {
    if (shifted_acc > HALFPOINT) return MAXPOINT;
    else return 0;
  }

  if (waveshape == SAW) {
    return shifted_acc;
  }

  if (waveshape == TRIANGLE) {
    if (shifted_acc <= HALFPOINT) {
      return (shifted_acc << 1);
    }
    else {
      return ((MAXPOINT - shifted_acc) << 1);
    }
  }

  if (waveshape == RANDOM) {
    if (shifted_acc < previous_acc[channel]) {
      randNum[channel] = random(MAXPOINT);
    }
    previous_acc[channel] = shifted_acc;
    return randNum[channel];
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////////
// LED intro show - runs once every time waveSelect wraps back to slot 1. //
// //
// IMPORTANT: the four front-panel LEDs are not on separate GPIO - the //
// firmware only ever configures pins 1/9/2/3 (the CV outputs themselves). //
// The LEDs sit on those same lines, so this animation is also a brief real //
// voltage excursion at whatever's patched into the jacks. That's expected //
// here (triggered by an explicit button press, not silently), same //
// trade-off the existing 2-second delay() on the long-press already makes. //
// //
// Panel jack numbering doesn't match CC register order (see the #1-#4 //
// comments in TCC0_Handler), so setPanelOutput() below translates panel //
// position -> the correct CC register for you. //
////////////////////////////////////////////////////////////////////////////////////
void setPanelOutput(int panelNum, unsigned int value)
{
  switch (panelNum) {
    case 1: REG_TCC0_CC1 = value; break;
    case 2: REG_TCC0_CC2 = value; break;
    case 3: REG_TCC0_CC3 = value; break;
    case 4: REG_TCC0_CC0 = value; break;
  }
}

void setAllPanelOutputs(unsigned int value)
{
  REG_TCC0_CC0 = value;
  REG_TCC0_CC1 = value;
  REG_TCC0_CC2 = value;
  REG_TCC0_CC3 = value;
}

void ledIntroShow()
{
  // Take manual control of the outputs - disable the ISR first so it can't
  // sneak live waveform data into the CC registers mid-animation.
  NVIC_DisableIRQ(TCC0_IRQn);

  // Double flash, all four together.
  for (int i = 0; i < 2; i++) {
    setAllPanelOutputs(511);
    delay(90);
    setAllPanelOutputs(0);
    delay(90);
  }

  // Chase 2 -> 3 -> 4 -> 1, two laps.
  int chaseOrder[4] = {2, 3, 4, 1};
  for (int lap = 0; lap < 2; lap++) {
    for (int i = 0; i < 4; i++) {
      setAllPanelOutputs(0);
      setPanelOutput(chaseOrder[i], 511);
      delay(110);
    }
  }
  setAllPanelOutputs(0);

  NVIC_EnableIRQ(TCC0_IRQn);
  // No further cleanup needed - the ISR picks back up with real waveform
  // data (TRIANGLE, since waveSelect==1 by the time this is called) on the
  // very next timer tick.
}

// NEW: maps a de Jong coordinate (roughly -2..+2) into the same 0-511 PWM
// range generator() uses, clamping in case a preset produces an excursion
// outside the nominal bound.
unsigned int scaleJong(float v) {
  float s = (v + 2.0f) * (511.0f / 4.0f);
  if (s < 0.0f) s = 0.0f;
  if (s > 511.0f) s = 511.0f;
  return (unsigned int)s;
}

void setupTimers() // used to set up fast PWM on pins 1,9,2,3
{
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(2) | GCLK_GENDIV_ID(4);
  while (GCLK->STATUS.bit.SYNCBUSY);

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(4);
  while (GCLK->STATUS.bit.SYNCBUSY);

  PORT->Group[g_APinDescription[1].ulPort].PINCFG[g_APinDescription[1].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[9].ulPort].PINCFG[g_APinDescription[9].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[3].ulPort].PINCFG[g_APinDescription[3].ulPin].bit.PMUXEN = 1;

  PORT->Group[g_APinDescription[1].ulPort].PMUX[g_APinDescription[1].ulPin >> 1].reg = PORT_PMUX_PMUXE_E;
  PORT->Group[g_APinDescription[9].ulPort].PMUX[g_APinDescription[9].ulPin >> 1].reg |= PORT_PMUX_PMUXO_E;
  PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F;
  PORT->Group[g_APinDescription[3].ulPort].PMUX[g_APinDescription[3].ulPin >> 1].reg |= PORT_PMUX_PMUXO_F;

  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC0_TCC1;
  while (GCLK->STATUS.bit.SYNCBUSY);

  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) | TCC_WAVE_WAVEGEN_DSBOTH | TCC_WAVE_WAVEGEN_NFRQ;
  while (TCC0->SYNCBUSY.bit.WAVE);

  REG_TCC0_PER = 0x1FF;
  while (TCC0->SYNCBUSY.bit.PER);

  REG_TCC0_CC1 = 10;
  while (TCC0->SYNCBUSY.bit.CC1);
  REG_TCC0_CC0 = 50;
  while (TCC0->SYNCBUSY.bit.CC0);
  REG_TCC0_CC2 = 200;
  while (TCC0->SYNCBUSY.bit.CC2);
  REG_TCC0_CC3 = 254;
  while (TCC0->SYNCBUSY.bit.CC3);

  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_ENABLE;
  while (TCC0->SYNCBUSY.bit.ENABLE);

  TCC0->INTENSET.reg = 0;
  TCC0->INTENSET.bit.CNT = 1;
  TCC0->INTENSET.bit.MC0 = 0;
  NVIC_EnableIRQ(TCC0_IRQn);

  TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE;
}

void TCC0_Handler()
{
  if (TCC0->INTFLAG.bit.CNT == 1) {
    if (waveSelect == DEJONG || waveSelect == SLOTH || waveSelect == STOOGES || waveSelect == CIPHER) {
      // No trig, no iterative solver here - just read out what loop() already
      // computed (de Jong, Sloth, Stooges, or the Cipher LFSR).
      REG_TCC0_CC0 = jongOut[0];
      REG_TCC0_CC1 = jongOut[1];
      REG_TCC0_CC2 = jongOut[2];
      REG_TCC0_CC3 = jongOut[3];
    }
    else if (waveSelect == DEJONG_SLEW) {
      // Cheap fixed-point one-pole slew toward jongOut[], every tick.
      // No trig, no float - just shifts and adds, safe at 46kHz.
      long target0 = (long)jongOut[0] << 16;
      long target1 = (long)jongOut[1] << 16;
      long target2 = (long)jongOut[2] << 16;
      long target3 = (long)jongOut[3] << 16;

      slewAccum[0] += (target0 - slewAccum[0]) >> SLEWSHIFT;
      slewAccum[1] += (target1 - slewAccum[1]) >> SLEWSHIFT;
      slewAccum[2] += (target2 - slewAccum[2]) >> SLEWSHIFT;
      slewAccum[3] += (target3 - slewAccum[3]) >> SLEWSHIFT;

      REG_TCC0_CC0 = (unsigned int)(slewAccum[0] >> 16);
      REG_TCC0_CC1 = (unsigned int)(slewAccum[1] >> 16);
      REG_TCC0_CC2 = (unsigned int)(slewAccum[2] >> 16);
      REG_TCC0_CC3 = (unsigned int)(slewAccum[3] >> 16);
    }
    else {
      accumulator1 = accumulator1 + phasor1;
      accumulator2 = accumulator2 + phasor2;
      accumulator3 = accumulator3 + phasor3;
      accumulator4 = accumulator4 + phasor4;

      REG_TCC0_CC0 = generator(accumulator1, waveSelect, 3); // pin 9
      REG_TCC0_CC1 = generator(accumulator4, waveSelect, 0); // pin 2
      REG_TCC0_CC2 = generator(accumulator2, waveSelect, 1); // pin 1
      REG_TCC0_CC3 = generator(accumulator3, waveSelect, 2); // pin 3
    }
    TCC0->INTFLAG.bit.CNT = 1;
  }
}

void readSettings(void)
{
  char x;
  char c = 'S';
  int y = 1;
  init_storage.read(x);
  if (x == 'S') {
    wave_storage.read(waveSelect);
    div_storage.read(divSelect);
  }
  else {
    init_storage.write(c);
    wave_storage.write(y);
    div_storage.write(y);
    divSelect = 1;
    waveSelect = 1;
  }
}

void saveSettings(void)
{
  int x;
  wave_storage.read(x);
  if (x != waveSelect) wave_storage.write(waveSelect);
  div_storage.read(x);
  if (x != divSelect) div_storage.write(divSelect);
}

#define NUMREADINGS 50
unsigned int pot[NUMREADINGS];
unsigned int freq[NUMREADINGS];

void filterPut(char input, unsigned int newreading)
{
  static unsigned char potptr = 0;
  static unsigned char freqptr = 0;
  if (input == POT) {
    pot[potptr] = newreading;
    potptr++;
    if (potptr >= NUMREADINGS) potptr = 0;
  }
  else if (input == FREQ) {
    freq[freqptr] = newreading;
    freqptr++;
    if (freqptr >= NUMREADINGS) freqptr = 0;
  }
}

unsigned int filterGet(bool input)
{
  unsigned long int x = 0;
  float z;
  unsigned char y;
  if (input == POT) {
    for (y = 0; y < NUMREADINGS; y++) x = x + pot[y];
  }
  else if (input == FREQ) {
    for (y = 0; y < NUMREADINGS; y++) x = x + freq[y];
  }
  z = x;
  z = z / NUMREADINGS;
  z = z + 0.5;
  return (unsigned int)z;
}
