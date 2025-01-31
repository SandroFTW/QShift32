/* TODO:
  - Rewrite so each Channel doesn't need its own timer
  - Implement volatile unsigned long lowStartTime[4] -> Prevent coils from overheating because of extended dwell time > ~200ms
  - Make RPM limiter code cleaner
*/

#include <WebSocketsServer.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebConfig.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>
#include <WiFi.h>

#include <Arduino.h>

WebSocketsServer webSocket = WebSocketsServer(81);

// Global variables
volatile unsigned long currTime          = 0;     // current timestamp in 1 ms steps (1 kHz)
volatile unsigned long dwellBeginTime[4] = {0};   // time of last dwell start in µs steps
volatile unsigned long lastDwellTime[4]  = {0};   // duration of last dwell pulse in µs steps
volatile unsigned long lastRPMdelta[4]   = {0};   // Stores time bewteen two ignition pulses, needed for RPM calculation
volatile bool lastMeasureState[4]        = {0};   // last state of the filtered coil state input
volatile bool shiftingTrig               = false; // Controls whether ignition is retarded
volatile bool limitingRPM                = false; // Temporarily changes restore time to 999 while RPM limiting
volatile int currRetard                  = 0;     // Used for gradual ignition retard recovery to normal operation

static unsigned long lastCycle           = 0;     // When was the last main loop cycle (1 kHz)
static unsigned long lastRead            = 0;     // When was the last ADC sensor reading (50 Hz)
static unsigned long lastCut             = 0;     // temporary R&D helper for on off toggling

static int currCutTime                   = 0;     // Interpolated cutTime based on current RPM and min/max time
static int lastRPM                       = 0;
static int pressureValue                 = 0;     // Piezo/Hall sensor pressure value
static bool buttonPressed                = false; // Handlebar button
static bool waitHyst                     = true;  // Needs to be low before another upshift is allowed

// Timer instances
hw_timer_t *t0_time = NULL;
hw_timer_t *t1_cut  = NULL;
hw_timer_t *t2_cut  = NULL;
hw_timer_t *t3_cut  = NULL;

// Pin definitions
const int HALL_PINS[2]    = {10, 11};
const int IGBT_PINS[4]    = {2, 3, 4, 1};
const int MEASURE_PINS[4] = {9, 14, 7, 8};

const int PIEZO_PIN = 13;
const int GPIO_PIN  = 12;
const int GREEN_PIN = 5;
const int RED_PIN   = 6;

String params = "["
  "{"
  "'name':'password',"
  "'label':'WiFi password',"
  "'type':"+String(INPUTPASSWORD)+","
  "'default':''"
  "},"
  "{"
  "'name':'retardMin',"
  "'label':'Ignition Retard - Low RPM (deg)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':180,"
  "'default':'80'"
  "},"
  "{"
  "'name':'retardMax',"
  "'label':'Ignition Retard - High RPM (deg)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':180,"
  "'default':'110'"
  "},"
  "{"
  "'name':'restore',"
  "'label':'Ignition Restore (deg/rev)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1,'max':9999,"
  "'default':'25'"
  "},"
  "{"
  "'name':'minRPM',"
  "'label':'Min RPM (1/min)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1000,'max':20000,"
  "'default':'3000'"
  "},"
  "{"
  "'name':'maxRPM',"
  "'label':'Max RPM (1/min)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':2000,'max':20000,"
  "'default':'12500'"
  "},"
  "{"
  "'name':'cutTimeMax',"
  "'label':'Cut Time - Low RPM (ms)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1,'max':200,"
  "'default':'54'"
  "},"
  "{"
  "'name':'cutTimeMin',"
  "'label':'Cut Time - High RPM (ms)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1,'max':200,"
  "'default':'48'"
  "},"
  "{"
  "'name':'deadTime',"
  "'label':'Dead Time (ms)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1,'max':5000,"
  "'default':'350'"
  "},"
  "{"
  "'name':'cutSens',"
  "'label':'Cut Sensitivity (1-4095)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1,'max':5000,"
  "'default':'1000'"
  "},"
  "{"
  "'name':'cutHyst',"
  "'label':'Cut Hysteresis (1-4095)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1,'max':5000,"
  "'default':'500'"
  "},"
  "{"
  "'name':'fullCut',"
  "'label':'Full Ignition Cut',"
  "'type':"+String(INPUTCHECKBOX)+","
  "'default':'0'"
  "},"
  "{"
  "'name':'wastedSpark',"
  "'label':'Wasted Spark Setup',"
  "'type':"+String(INPUTCHECKBOX)+","
  "'default':'1'"
  "},"
  "{"
  "'name':'limiterButton',"
  "'label':'Limiter Push Button',"
  "'type':"+String(INPUTCHECKBOX)+","
  "'default':'1'"
  "},"
  "{"
  "'name':'limiterRPM',"
  "'label':'Limiter RPM (1/min)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':20000,"
  "'default':'6000'"
  "},"
  "{"
  "'name':'limiterCut',"
  "'label':'Limiter Cut Gain (ms/100rpm)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1,'max':1000,"
  "'default':'8'"
  "},"
  "{"
  "'name':'limiterRetard',"
  "'label':'Limiter Retard (deg)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':180,"
  "'default':'50'"
  "}"
  "]";

AsyncWebServer server(80);
AsyncWebConfig conf;

struct cfgOptions
{
  char ssid[32];
  char password[32];

  float retardMin;
  float retardMax;
  int restore;

  int minRPM;
  int maxRPM;

  int cutTimeMax;
  int cutTimeMin;

  int deadTime;
  int cutSens;
  int cutHyst;

  bool fullCut;
  int wastedSpark;

  bool limiterButton;
  int limiterRPM;
  int limiterCut;
  int limiterRetard;
} cfg;

void transferWebconfToStruct(String results)
{
  cfg.retardMin = conf.getInt("retardMin");
  cfg.retardMax = conf.getInt("retardMax"); 
  cfg.restore = conf.getInt("restore");        // TODO: Think how this is affected by cylinder/coil count with wasted spark          // TODO: does this also need to be float and just subtracted from retard value?

  cfg.minRPM = conf.getInt("minRPM");
  cfg.maxRPM = conf.getInt("maxRPM");

  cfg.cutTimeMax = conf.getInt("cutTimeMax");
  cfg.cutTimeMin = conf.getInt("cutTimeMin");

  cfg.deadTime = conf.getInt("deadTime");
  cfg.cutSens = conf.getInt("cutSens");
  cfg.cutHyst = conf.getInt("cutHyst");

  cfg.fullCut = conf.getBool("fullCut");
  cfg.wastedSpark = conf.getBool("wastedSpark") ? 1 : 0;

  cfg.limiterButton = conf.getBool("limiterButton");
  cfg.limiterRPM = conf.getInt("limiterRPM");
  cfg.limiterCut = conf.getInt("limiterCut");
  cfg.limiterRetard = conf.getInt("limiterRetard");
}

const char debug_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>QShift32 Debug</title>
</head>
<body>
  <h2>QShift32 Debug</h2>
  <p id="val1">...</p>
  <p id="val2">...</p>
  <p id="val3">...</p>
  <p id="val4">...</p>
  <p id="val5">...</p>
  <script>
    const valEl1 = document.getElementById('val1');
    const valEl2 = document.getElementById('val2');
    const valEl3 = document.getElementById('val3');
    const valEl4 = document.getElementById('val4');
    const valEl5 = document.getElementById('val5');
    const ws = new WebSocket(`ws://${location.hostname}:81/`);

    ws.onmessage = function(event) {
      const [val1, val2, val3, val4, val5] = event.data.split(",");
      valEl1.innerText = `Pressure: ${val1}`;
      valEl2.innerText = `RPM: ${val2}`;
      valEl3.innerText = `Last Dwell [0]: ${val3}`;
      valEl4.innerText = `targetRetard: ${val4}`;
      valEl5.innerText = `currentCutTime: ${val5}`;
    };
  </script>
</body>
</html>
)rawliteral";

void handleRoot(AsyncWebServerRequest *request)
{
  conf.handleFormRequest(request);
}

// Create a task that will be executed on Core 0 (instead of 1)
TaskHandle_t Task1;
void push_debug(void *parameters)
{
  for (;;)
  {
    // pressure, rpm, lastdwell, xtal_time
    String broadcastString = String(pressureValue) + "," + String(lastRPM) + "," + String(lastDwellTime[0] * 10) + "," + String(currRetard) + "," + String(currCutTime);

    webSocket.loop();
    webSocket.broadcastTXT(broadcastString);

    delay(100);
  }
}



// Count in 1 ms steps (1 kHz)
void IRAM_ATTR on_t0_time()
{
  currTime++;
}

// Gets executed when (lastDwell + calculated delay) passed after shifting
void IRAM_ATTR on_t1_cut()
{
  digitalWrite(IGBT_PINS[0], HIGH);
}
void IRAM_ATTR on_t2_cut()
{
  digitalWrite(IGBT_PINS[1], HIGH);
}
void IRAM_ATTR on_t3_cut()
{
  digitalWrite(IGBT_PINS[2], HIGH);
}

// Channel 1 - When ECU changes state of coil (pull low or release)
void IRAM_ATTR onPC0()
{
  bool bPin = digitalRead(MEASURE_PINS[0]); // Measurement Pin
  unsigned long lTime = currTime;

  // When ECU pulls coil to ground (rising edge on logic signal)
  if (bPin && !lastMeasureState[0])
  {
    lastRPMdelta[0] = lTime - dwellBeginTime[0];

    unsigned int delayForRetard = (currRetard / (cfg.wastedSpark ? 360.f : 720.f)) * (lTime - dwellBeginTime[0]); // TODO: optimize this

    if (!shiftingTrig && currRetard > 0)                                             // TODO: Needed on all 4 channels or only when channel 1 triggers?
      currRetard = max(0, currRetard - cfg.restore);

    dwellBeginTime[0] = lTime;

    // Either actively retarding ignition or recovering
    if (currRetard > 0)
    {
      if (shiftingTrig && cfg.fullCut)
      {
        delayForRetard += ((currCutTime*100 / lastRPMdelta[0]) + 1) * lastRPMdelta[0];
        shiftingTrig = false;
      }

      digitalWrite(IGBT_PINS[0], LOW); // Close IGBT
      timerWrite(t1_cut, 0);
      timerAlarm(t1_cut, lastDwellTime[0] + delayForRetard, false, 0);
    }
  }
  // When ECU releases coil (falling edge on logic signal)
  else if (!bPin && !shiftingTrig && currRetard == 0 && dwellBeginTime[0])
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
    lastRPMdelta[1] = lTime - dwellBeginTime[1];

    unsigned int delayForRetard = (currRetard / (cfg.wastedSpark ? 360.f : 720.f)) * (lTime - dwellBeginTime[1]);
    dwellBeginTime[1] = lTime;

    if (currRetard > 0)
    {
      if (shiftingTrig && cfg.fullCut)
      {
        delayForRetard += ((currCutTime*100 / lastRPMdelta[1]) + 1) * lastRPMdelta[1];
        shiftingTrig = false;
      }

      digitalWrite(IGBT_PINS[1], LOW); // Close IGBT
      timerWrite(t2_cut, 0);
      timerAlarm(t2_cut, lastDwellTime[1] + delayForRetard, false, 0);
    }
  }
  // When ECU releases coil (falling edge on logic signal)
  else if (!bPin && !shiftingTrig && currRetard == 0 && dwellBeginTime[1])
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
    lastRPMdelta[2] = lTime - dwellBeginTime[2];

    unsigned int delayForRetard = (currRetard / (cfg.wastedSpark ? 360.f : 720.f)) * (lTime - dwellBeginTime[2]);
    dwellBeginTime[2] = lTime;

    if (currRetard > 0)
    {
      if (shiftingTrig && cfg.fullCut)
      {
        delayForRetard += ((currCutTime*100 / lastRPMdelta[2]) + 1) * lastRPMdelta[2];
        shiftingTrig = false;
      }

      digitalWrite(IGBT_PINS[2], LOW); // Close IGBT
      timerWrite(t3_cut, 0);
      timerAlarm(t3_cut, lastDwellTime[2] + delayForRetard, false, 0);
    }
  }
  // When ECU releases coil (falling edge on logic signal)
  else if (!bPin && !shiftingTrig && currRetard == 0 && dwellBeginTime[2])
  {
    lastDwellTime[2] = lTime - dwellBeginTime[2];
  }

  lastMeasureState[2] = bPin;
}

// ########## DISABLED FOR NOW ##########
// Channel 4 - When ECU changes state of coil (pull low or release)
// void IRAM_ATTR onPC3()
// {
//   bool bPin = digitalRead(MEASURE_PINS[3]); // Measurement Pin
//   unsigned long lTime = currTime;

//   // When ECU pulls coil to ground (rising edge on logic signal)
//   if (bPin && !lastMeasureState[3])
//   {
//     unsigned int delayForRetard = (currRetard / 360.f) * (lTime - dwellBeginTime[3]);
//     dwellBeginTime[3] = lTime;

//     if (shiftingTrig || currRetard > 0)
//     {
//       digitalWrite(IGBT_PINS[3], LOW); // Close IGBT
//       timerWrite(t1_cut, 0);
//       timerAlarm(t1_cut, lastDwellTime[3] + delayForRetard, false, 0);
//     }
//   }
//   // When ECU releases coil (falling edge on logic signal)
//   else if (!bPin && !shiftingTrig && currRetard == 0 && dwellBeginTime[3])
//   {
//     lastDwellTime[3] = lTime - dwellBeginTime[3];
//   }

//   lastMeasureState[3] = bPin;
// }

void setup()
{
  Serial.begin(57600);

  // LED
  pinMode(GREEN_PIN, OUTPUT);
  digitalWrite(GREEN_PIN, HIGH); // Off default
  pinMode(RED_PIN, OUTPUT);
  digitalWrite(RED_PIN, HIGH); // Off default

  // Hall, Piezo, GPIO
  int adcPins[4] = {HALL_PINS[0], HALL_PINS[1], PIEZO_PIN, GPIO_PIN};
  for (int i = 0; i < 4; i++) {
    pinMode(adcPins[i], INPUT);
  }

  // Ignition IGBT and measurement (set all 4 channels)
  for (int i = 0; i < 4; i++) {
    pinMode(IGBT_PINS[i], OUTPUT);
    digitalWrite(IGBT_PINS[i], HIGH); // Open IGBT default

    pinMode(MEASURE_PINS[i], INPUT_PULLUP);                 // TODO: Does this need to be pullup?
  }

  // Timer 0, Time, 100 kHz -> 10 µs resolution
  t0_time = timerBegin(100000);
  timerAttachInterrupt(t0_time, &on_t0_time);
  timerAlarm(t0_time, 1, true, 0); // Trigger interrupt every 10 µs (100 kHz)

  // Timer 1, Cut, 100 kHz -> 10 µs resolution
  t1_cut = timerBegin(100000);
  timerAttachInterrupt(t1_cut, &on_t1_cut);

  // Timer 2, Cut, 100 kHz -> 10 µs resolution
  t2_cut = timerBegin(100000);
  timerAttachInterrupt(t2_cut, &on_t2_cut);

  // Timer 3, Cut, 100 kHz -> 10 µs resolution
  t3_cut = timerBegin(100000);
  timerAttachInterrupt(t3_cut, &on_t3_cut);

  // Attach interrupts to MEASURE_PINS[i]
  void (*pcFuncs[4])() = {onPC0, onPC1, onPC2}; // onPC3
  for (int i = 0; i < 3; i++) { // i < 4
    attachInterrupt(digitalPinToInterrupt(MEASURE_PINS[i]), pcFuncs[i], CHANGE);
  }

  conf.setDescription(params);
  conf.readConfig();
  transferWebconfToStruct("");

  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_7dBm);
  WiFi.softAP(conf.getApName(), conf.values[0].c_str());

  server.on("/", handleRoot);

  server.on("/debug.html", HTTP_GET, [](AsyncWebServerRequest *request) {
    //request->send(SPIFFS, "/debug.html", "text/html");
    request->send_P(200, "text/html", debug_html);
  });

  conf.registerOnSave(transferWebconfToStruct);

  server.begin();
  webSocket.begin();

  xTaskCreatePinnedToCore(push_debug, "CPU_0", 10000, NULL, 1, &Task1, 0);
}

void loop()
{
  // Run every 1 ms (1 kHz)
  if ((currTime - lastCycle) >= 100)
  {
    // Read ADC sensors every 20 ms (50 Hz)
    if ((currTime - lastRead) >= 2000)
    {
      pressureValue = analogRead(HALL_PINS[0]);
      buttonPressed = !digitalRead(HALL_PINS[1]);
      //Serial.println(pressureValue);
      if (lastRPMdelta[0] > 0)
        lastRPM = (cfg.wastedSpark ? 6000000UL : 12000000UL) / lastRPMdelta[0];

      lastRead = currTime;
    }

    if (pressureValue < (cfg.cutSens - cfg.cutHyst))
      waitHyst = false;

    if (lastRPM < cfg.minRPM)
      waitHyst = true;

    // 2-step launch control
    if (cfg.limiterRPM && cfg.limiterCut && cfg.limiterRetard)
    {
      if ((!cfg.limiterButton || buttonPressed) && !shiftingTrig)
      {
        if (lastRPM > cfg.limiterRPM)
        {
          limitingRPM = true;
          shiftingTrig = true;
          waitHyst = true;
          currRetard = cfg.limiterRetard;
          currCutTime = (int)((lastRPM - cfg.limiterRPM) * cfg.limiterCut / 100.f); // rpmError * gain
          cfg.restore = (int)(cfg.limiterRetard / 2);
          cfg.fullCut = true;
        }
      }
      else
      {
        limitingRPM = false;
        shiftingTrig = false;
        cfg.restore = conf.getInt("restore");
        cfg.fullCut = conf.getBool("fullCut");
      }
    }

    if (pressureValue > cfg.cutSens && lastRPM >= cfg.minRPM && !waitHyst && !shiftingTrig && !limitingRPM && (currTime - lastCut) >= cfg.deadTime*100)
    {
      shiftingTrig = true;
      currRetard = map(lastRPM, cfg.minRPM, cfg.maxRPM, cfg.retardMin, cfg.retardMax);
      currCutTime = map(lastRPM, cfg.minRPM, cfg.maxRPM, cfg.cutTimeMax, cfg.cutTimeMin);

      waitHyst = true;

      digitalWrite(GREEN_PIN, LOW); // On when shifting

      lastCut = currTime;
    }

    if (shiftingTrig && !limitingRPM && (currTime - lastCut) >= currCutTime*100)
    {
      shiftingTrig = false;
      digitalWrite(GREEN_PIN, HIGH); // Off
    }

    lastCycle = currTime;
  }
}
