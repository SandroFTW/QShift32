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
volatile int currRestore                 = 0;     // Calculated deg/ignition pulse recovery speed

static unsigned long lastCycle           = 0;     // When was the last main loop cycle (1 kHz)
static unsigned long lastRead            = 0;     // When was the last ADC sensor reading (50 Hz)
static unsigned long lastCut             = 0;     // temporary R&D helper for on off toggling

static int currHoldTime                  = 0;     // Interpolated hold time based on current RPM and low/high time
static int lastRPM                       = 0;
static int pressureValue                 = 0;      // Piezo/Hall sensor pressure value
static bool buttonPressed                = false;  // Handlebar button
static bool waitHyst                     = true;   // Needs to be low before another upshift is allowed

// Timer instances
hw_timer_t *t0_time = NULL;
hw_timer_t *t_cut[3]  = {NULL, NULL, NULL};

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
  "'name':'retardLow',"
  "'label':'Ignition Retard - Low RPM (deg)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':180,"
  "'default':'40'"
  "},"
  "{"
  "'name':'retardHigh',"
  "'label':'Ignition Retard - High RPM (deg)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':180,"
  "'default':'50'"
  "},"
  "{"
  "'name':'restore',"
  "'label':'Restore Smoothness (Ign Pulses)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':1000,"
  "'default':'10'"
  "},"
  "{"
  "'name':'minRPM',"
  "'label':'Min RPM (1/min)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1000,'max':20000,"
  "'default':'2950'"
  "},"
  "{"
  "'name':'maxRPM',"
  "'label':'Max RPM (1/min)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':2000,'max':20000,"
  "'default':'12500'"
  "},"
  "{"
  "'name':'holdTimeLow',"
  "'label':'Cut Time - Low RPM (ms)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1,'max':1000,"
  "'default':'44'"
  "},"
  "{"
  "'name':'holdTimeHigh',"
  "'label':'Cut Time - High RPM (ms)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1,'max':1000,"
  "'default':'50'"
  "},"
  "{"
  "'name':'deadTime',"
  "'label':'Dead Time (ms)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1,'max':5000,"
  "'default':'300'"
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
  "'default':'1'"
  "},"
  "{"
  "'name':'wastedSpark',"
  "'label':'Wasted Spark Setup',"
  "'type':"+String(INPUTCHECKBOX)+","
  "'default':'1'"
  "},"
  "{"
  "'name':'limiterButton',"
  "'label':'Limiter Require Button',"
  "'type':"+String(INPUTCHECKBOX)+","
  "'default':'1'"
  "},"
  "{"
  "'name':'limiterFullCut',"
  "'label':'Limiter Full Cut',"
  "'type':"+String(INPUTCHECKBOX)+","
  "'default':'1'"
  "},"
  "{"
  "'name':'limiterRPM',"
  "'label':'Limiter RPM (1/min)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':20000,"
  "'default':'3500'"
  "},"
  "{"
  "'name':'limiterCut',"
  "'label':'Limiter Cut Gain (ms/100rpm)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1,'max':1000,"
  "'default':'10'"
  "},"
  "{"
  "'name':'limiterRetard',"
  "'label':'Limiter Retard (deg)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':180,"
  "'default':'35'"
  "},"
  "{"
  "'name':'limiterDiv',"
  "'label':'Limiter Restore Divisor',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1,'max':20,"
  "'default':'2'"
  "}"
  "]";

AsyncWebServer server(80);
AsyncWebConfig conf;

struct cfgOptions
{
  char ssid[32];
  char password[32];

  int retardLow;
  int retardHigh;
  int restore;

  int minRPM;
  int maxRPM;

  int holdTimeLow;
  int holdTimeHigh;

  int deadTime;
  int cutSens;
  int cutHyst;

  bool fullCut;
  float wastedSpark;

  bool limiterButton;
  bool limiterFullCut;
  int limiterRPM;
  int limiterCut;
  int limiterRetard;
  int limiterDiv;
} cfg;

void transferWebconfToStruct(String results)
{
  cfg.retardLow = conf.getInt("retardLow");
  cfg.retardHigh = conf.getInt("retardHigh"); 
  cfg.restore = conf.getInt("restore");        // TODO: Think how this is affected by cylinder/coil count with wasted spark          // TODO: does this also need to be float and just subtracted from retard value?

  cfg.minRPM = conf.getInt("minRPM");
  cfg.maxRPM = conf.getInt("maxRPM");

  cfg.holdTimeLow = conf.getInt("holdTimeLow");
  cfg.holdTimeHigh = conf.getInt("holdTimeHigh");

  cfg.deadTime = conf.getInt("deadTime");
  cfg.cutSens = conf.getInt("cutSens");
  cfg.cutHyst = conf.getInt("cutHyst");

  cfg.fullCut = conf.getBool("fullCut");
  cfg.wastedSpark = conf.getBool("wastedSpark") ? 360.f : 720.f;      // TODO: Rename maybe?

  cfg.limiterButton = conf.getBool("limiterButton");
  cfg.limiterFullCut = conf.getBool("limiterFullCut");
  cfg.limiterRPM = conf.getInt("limiterRPM");
  cfg.limiterCut = conf.getInt("limiterCut");
  cfg.limiterRetard = conf.getInt("limiterRetard");
  cfg.limiterDiv = conf.getInt("limiterDiv");
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
  <p id="val6">...</p>
  <p id="val7">...</p>
  <script>
    const valEl1 = document.getElementById('val1');
    const valEl2 = document.getElementById('val2');
    const valEl3 = document.getElementById('val3');
    const valEl4 = document.getElementById('val4');
    const valEl5 = document.getElementById('val5');
    const valEl6 = document.getElementById('val6');
    const valEl7 = document.getElementById('val7');
    const ws = new WebSocket(`ws://${location.hostname}:81/`);

    ws.onmessage = function(event) {
      const [val1, val2, val3, val4, val5, val6, val7] = event.data.split(",");
      valEl1.innerText = `Pressure: ${val1}`;
      valEl2.innerText = `RPM: ${val2}`;
      valEl3.innerText = `Last Dwell [0]: ${val3}`;
      valEl4.innerText = `targetRetard: ${val4}`;
      valEl5.innerText = `currHoldTime: ${val5}`;
      valEl6.innerText = `currRestore: ${val6}`;
      valEl7.innerText = `shiftingTrig: ${val7}`;
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
    String broadcastString = String(pressureValue) + "," + String(lastRPM) + "," + String(lastDwellTime[0] * 10) + "," + String(currRetard) + "," + String(currHoldTime) + "," + String(currRestore) + "," + (shiftingTrig ? "true" : "false");

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

// When ECU changes state of coil (pull low or release)
void coilInterrupt(int ch)
{
  bool bPin = digitalRead(MEASURE_PINS[ch]); // Measurement Pin
  unsigned long lTime = currTime;

  // When ECU pulls coil to ground (rising edge on logic signal)
  if (bPin && !lastMeasureState[ch])
  {
    lastRPMdelta[ch] = lTime - dwellBeginTime[ch];

    unsigned int delayForRetard = (currRetard / 360.f) * (lTime - dwellBeginTime[ch]); // TODO: optimize this // cfg.wastedSpark

    if (!shiftingTrig && currRetard > 0) {                                             // TODO: Needed on all 4 channels or only when channel 1 triggers?
      currRetard = max(0, currRetard - currRestore);
    }

    dwellBeginTime[ch] = lTime;

    // Either actively retarding ignition or recovering
    if (currRetard > 0)
    {
      if (shiftingTrig && cfg.fullCut)
      {
        delayForRetard += ((currHoldTime*100 / lastRPMdelta[ch]) + 1) * lastRPMdelta[ch];
        shiftingTrig = false;
      }

      digitalWrite(IGBT_PINS[ch], LOW); // Close IGBT
      timerWrite(t_cut[ch], 0);
      timerAlarm(t_cut[ch], lastDwellTime[ch] + delayForRetard, false, 0);
    }
  }
  // When ECU releases coil (falling edge on logic signal)
  else if (!bPin && !shiftingTrig && currRetard == 0 && dwellBeginTime[ch])
  {
    lastDwellTime[ch] = lTime - dwellBeginTime[ch];
  }

  lastMeasureState[ch] = bPin;
}

void IRAM_ATTR onPC0() { coilInterrupt(0); }
void IRAM_ATTR onPC1() { coilInterrupt(1); }
void IRAM_ATTR onPC2() { coilInterrupt(2); }

// ########## DISABLED FOR NOW ##########
// Channel 4

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

  void (*on_cut_funcs[4])() = {on_t1_cut, on_t2_cut, on_t3_cut};
  for (int i = 0; i < 3; i++) {
    t_cut[i] = timerBegin(100000);
    timerAttachInterrupt(t_cut[i], on_cut_funcs[i]);
  }

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
          currHoldTime = min((int)((lastRPM - cfg.limiterRPM) * cfg.limiterCut / 100.f), 100); // rpmError * gain
          currRestore = (int)(cfg.limiterRetard / cfg.limiterDiv);
          cfg.fullCut = cfg.limiterFullCut;
        }
      }
      else
      {
        if (limitingRPM)
        {
          limitingRPM = false;
          shiftingTrig = false;   // ############################## OHHHH MANNNNNNN
          cfg.fullCut = conf.getBool("fullCut");
        }
      }
    }

    if (pressureValue > cfg.cutSens && lastRPM >= cfg.minRPM && !waitHyst && !shiftingTrig && !limitingRPM && (currTime - lastCut) >= cfg.deadTime*100)
    {
      shiftingTrig = true;
      currRetard = map(lastRPM, cfg.minRPM, cfg.maxRPM, cfg.retardLow, cfg.retardHigh);                 // TODO: Make this logarithmic instead of linear (more change at low rpm, less change at high rpm)
      currHoldTime = map(lastRPM, cfg.minRPM, cfg.maxRPM, cfg.holdTimeLow, cfg.holdTimeHigh);

      currRestore = (int)(currRetard / cfg.restore) + 1;

      waitHyst = true;

      digitalWrite(GREEN_PIN, LOW); // On when shifting

      lastCut = currTime;
    }

    if (shiftingTrig && !limitingRPM && (currTime - lastCut) >= currHoldTime*100)
    {
      shiftingTrig = false;
      digitalWrite(GREEN_PIN, HIGH); // Off
    }

    lastCycle = currTime;
  }
}
