/* TODO:
  - Use multiple timers for 4 ignition channels to prevent issues because of dwell time overlap
  - Implement volatile unsigned long lowStartTime[4] -> Prevent coils from overheating because of extended dwell time > ~200ms
*/

#include <Arduino.h>

volatile unsigned long currTime          = 0;
volatile unsigned long dwellBeginTime[4] = {0};
volatile unsigned long lastDwellTime[4]  = {0};
volatile bool lastMeasureState[4]        = {0};
volatile int lastCutChannel              = -1;
volatile bool shiftingTrig               = false;

static unsigned long lastCycle = 0; // When was the last main loop cycle (1 kHz)
static unsigned long lastCut   = 0; // temporary R&D helper for on off toggling

// Timer instances
hw_timer_t *t0_time = NULL;
hw_timer_t *t1_cut  = NULL;

// Pin definitions
const int HALL_PINS[2]    = {10, 11};
const int IGBT_PINS[4]    = {2, 3, 4, 1};
const int MEASURE_PINS[4] = {9, 14, 7, 8};

const int PIEZO_PIN = 13;
const int GREEN_PIN = 5;
const int RED_PIN   = 6;
const int GPIO      = 12;

// Count in 10 us steps (100 kHz)
void IRAM_ATTR on_t0_time()
{
  currTime++;
}

// Gets executed when (lastDwell + calculated delay) passed after shifting
void IRAM_ATTR on_t1_cut()
{
  //timerAlarmDisable(t0_cut);                         // TODO: Check if this is needed
  digitalWrite(IGBT_PINS[lastCutChannel], HIGH);
}

// Channel 1 - When ECU changes state of coil (pull low or release)
void IRAM_ATTR onPC0()
{
  bool bPin = digitalRead(MEASURE_PINS[0]); // Measurement Pin
  unsigned long lTime = currTime;

  // When ECU pulls coil to ground (rising edge on logic signal)
  if (bPin && !lastMeasureState[0])
  {
    dwellBeginTime[0] = lTime;

    if (shiftingTrig)
    {
      digitalWrite(IGBT_PINS[0], LOW); // Close IGBT
      lastCutChannel = 0;
      timerWrite(t1_cut, 0);
      timerAlarm(t1_cut, lastDwellTime[0] + 100, false, 0);
    }
  }
  // When ECU releases coil (falling edge on logic signal)
  else if (!bPin && !shiftingTrig && dwellBeginTime[0])
  {
    lastDwellTime[0] = lTime - dwellBeginTime[0];
  }

  lastMeasureState[0] = bPin;
}

// Channel 2 - When ECU changes state of coil (pull low or release)
void IRAM_ATTR onPC1()
{
  bool bPin = digitalRead(MEASURE_PINS[1]); // Measurement Pin
  unsigned long lTime = currTime;

  // When ECU pulls coil to ground (rising edge on logic signal)
  if (bPin && !lastMeasureState[1])
  {
    dwellBeginTime[1] = lTime;

    if (shiftingTrig)
    {
      digitalWrite(IGBT_PINS[1], LOW); // Close IGBT
      lastCutChannel = 1;
      timerWrite(t1_cut, 0);
      timerAlarm(t1_cut, lastDwellTime[1] + 100, false, 0);
    }
  }
  // When ECU releases coil (falling edge on logic signal)
  else if (!bPin && !shiftingTrig && dwellBeginTime[1])
  {
    lastDwellTime[1] = lTime - dwellBeginTime[1];
  }

  lastMeasureState[1] = bPin;
}

// Channel 3 - When ECU changes state of coil (pull low or release)
void IRAM_ATTR onPC2()
{
  bool bPin = digitalRead(MEASURE_PINS[2]); // Measurement Pin
  unsigned long lTime = currTime;

  // When ECU pulls coil to ground (rising edge on logic signal)
  if (bPin && !lastMeasureState[2])
  {
    dwellBeginTime[2] = lTime;

    if (shiftingTrig)
    {
      digitalWrite(IGBT_PINS[2], LOW); // Close IGBT
      lastCutChannel = 2;
      timerWrite(t1_cut, 0);
      timerAlarm(t1_cut, lastDwellTime[2] + 100, false, 0);
    }
  }
  // When ECU releases coil (falling edge on logic signal)
  else if (!bPin && !shiftingTrig && dwellBeginTime[2])
  {
    lastDwellTime[2] = lTime - dwellBeginTime[2];
  }

  lastMeasureState[2] = bPin;
}

// Channel 4 - When ECU changes state of coil (pull low or release)
void IRAM_ATTR onPC3()
{
  bool bPin = digitalRead(MEASURE_PINS[3]); // Measurement Pin
  unsigned long lTime = currTime;

  // When ECU pulls coil to ground (rising edge on logic signal)
  if (bPin && !lastMeasureState[3])
  {
    dwellBeginTime[3] = lTime;

    if (shiftingTrig)
    {
      digitalWrite(IGBT_PINS[3], LOW); // Close IGBT
      lastCutChannel = 3;
      timerWrite(t1_cut, 0);
      timerAlarm(t1_cut, lastDwellTime[3] + 100, false, 0);
    }
  }
  // When ECU releases coil (falling edge on logic signal)
  else if (!bPin && !shiftingTrig && dwellBeginTime[3])
  {
    lastDwellTime[3] = lTime - dwellBeginTime[3];
  }

  lastMeasureState[3] = bPin;
}

void setup()
{
  // LED
  pinMode(GREEN_PIN, OUTPUT);
  digitalWrite(GREEN_PIN, HIGH); // Off default
  pinMode(RED_PIN, OUTPUT);
  digitalWrite(RED_PIN, HIGH); // Off default

  // Hall, Piezo, GPIO
  int adcPins[4] = {HALL_PINS[0], HALL_PINS[1], PIEZO_PIN, GPIO};
  for (int i = 0; i < 4; i++) {
    pinMode(adcPins[i], INPUT);
  }

  // Ignition IGBT and measurement (set all 4 channels)
  for (int i = 0; i < 4; i++) {
    pinMode(IGBT_PINS[i], OUTPUT);
    digitalWrite(IGBT_PINS[i], HIGH); // Open IGBT default

    pinMode(MEASURE_PINS[i], INPUT_PULLUP);                 // TODO: Does this need to be pullup?
  }

  // timer 1, 100 kHz -> 10 us resolution
  t0_time = timerBegin(100000);
  timerAttachInterrupt(t0_time, &on_t0_time);
  timerAlarm(t0_time, 1, true, 0); // Trigger interrupt every 10 us (100 kHz)

  // Timer 0 configuration for coil firing
  t1_cut = timerBegin(100000); // 100 kHz -> 10 us resolution
  timerAttachInterrupt(t1_cut, &on_t1_cut);

  // Attach interrupts to MEASURE_PINS[i]
  void (*pcFuncs[4])() = {onPC0, onPC1, onPC2, onPC3};
  for (int i = 0; i < 4; i++) {
    attachInterrupt(digitalPinToInterrupt(MEASURE_PINS[i]), pcFuncs[i], CHANGE);
  }
}

void loop()
{
  while (true)
  {
    // Run every 1 ms (1 kHz)
    if (currTime - lastCycle >= 100)
    {
      // Run every 0.2 s (5 Hz)
      if (currTime - lastCut >= 20000)
      {
        shiftingTrig = !shiftingTrig;
        digitalWrite(GREEN_PIN, !shiftingTrig); // On when shifting

        lastCut = currTime;
      }

      lastCycle = currTime;
    }
  }
}
