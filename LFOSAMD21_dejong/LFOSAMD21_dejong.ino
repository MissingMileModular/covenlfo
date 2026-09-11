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
#define DEJONG   5   // NEW

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
        if (waveSelect > 5) { // NEW: was >4, now includes DEJONG
          waveSelect = 1;
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
  // De Jong attractor iteration (only runs while waveSelect == DEJONG) //
  // Iterates at sweepValue Hz (same pot/CV curve everything else uses), //
  // clamped so sinf/cosf never has to run faster than the loop can keep up. //
  ////////////////////////////////////////////////////////////////////////////////////
  if (waveSelect == DEJONG) {
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
    if (waveSelect == DEJONG) {
      // No trig here - just read out what loop() already computed.
      REG_TCC0_CC0 = jongOut[0];
      REG_TCC0_CC1 = jongOut[1];
      REG_TCC0_CC2 = jongOut[2];
      REG_TCC0_CC3 = jongOut[3];
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
