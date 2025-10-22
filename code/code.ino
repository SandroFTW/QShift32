/* TODO:
  - Maybe add some kind of detection for cylinder count (of course only while engine is running) so cfg.restore gets scaled accordingly (makes config more generally usable)
  - Rewrite so each Channel doesn't need its own timer
    -> meh, semi fixed by removing the need for a seperate timekeeping timer so all 4 timers can be used for qs channels
  - Implement volatile unsigned long lowStartTime[4] -> Prevent coils from overheating because of extended dwell time > ~200ms
  - Think how cfg.restore is affected by cylinder/coil count with wasted spark (double the cylinder count needs double cfg.restore for same smoothness, change to float for more precision or unnecessary?)
  - Rename cfg.wastedSpark to something that describes it better (crank degrees per ignition pulse per coil) (Change back to boolean, create global variable that stores 360 or 720 for coilInterrupt function but keeps this simple)
  - Try to improve performance of coilInterrupt interrupt function (is micros() bad? less calculations)
  - Make currRetard calculation logarithmic instead of linear (more change at low rpm, less change at high rpm, with configurable breakpoint percentage, maybe see throttle expo in betaflight)
*/

#include <WebSocketsServer.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebConfig.h>
#include <esp_wifi.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>
#include <WiFi.h>

#include <Arduino.h>

#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"
#include "driver/gpio.h"

WebSocketsServer webSocket = WebSocketsServer(81);

// Global variables
volatile unsigned long currTime          = 0;     // current timestamp in 1 µs steps (1 MHz)
volatile unsigned long dwellBeginTime[4] = {0};   // time of last dwell start in µs steps
volatile unsigned long lastDwellTime[4]  = {0};   // duration of last dwell pulse in µs steps
volatile unsigned long lastRPMdelta[4]   = {0};   // Stores time bewteen two ignition pulses, needed for RPM calculation
volatile bool lastMeasureState[4]        = {0};   // last state of the filtered coil state input
volatile bool shiftingTrig               = false; // Controls whether ignition is retarded
volatile bool limitingRPM                = false; // Temporarily changes restore time to 999 while RPM limiting
volatile int currRetard                  = 0;     // Used for gradual ignition retard recovery to normal operation
volatile int currRestore                 = 0;     // Calculated deg/ignition pulse recovery speed

unsigned long lastCycle           = 0;     // When was the last main loop cycle (1 kHz)
unsigned long lastRead            = 0;     // When was the last ADC sensor reading (20 Hz)
unsigned long lastCut             = 0;     // When was the last ignition cut/retard begin

int currHoldTime                  = 0;     // Interpolated hold time based on current RPM and low/high time
int currStag                      = 0;     // Backup the cfg.staggeredCut value for toggling via button
int lastRPM                       = 0;
int pressureValue                 = 0;     // Piezo/Hall sensor pressure value
int limiterState                  = 0;     // Current state of RPM limiter mode
bool buttonPressed                = false; // Handlebar button state
bool lastButtonState              = false; // Last button state for edge detection
bool waitHyst                     = true;  // Needs to be low before another upshift is allowed

// Timer instances
hw_timer_t *t_cut[4]  = {NULL, NULL, NULL, NULL};

// Pin definitions
const int HALL_PINS[2]    = {10, 11};
const int IGBT_PINS[4]    = {2, 3, 4, 1};
const int MEASURE_PINS[4] = {9, 14, 7, 8}; // Handwired first revision (xj6)
//const int MEASURE_PINS[4] = {9, 7, 8, 12}; // PCB Design Pins (grom, R3b)

const int PIEZO_PIN = 13;
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
  "'name':'category1',"
  "'label':'Quickshifter Settings',"
  "'type':"+String(CATEGORY)+""
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
  "'default':'20'"
  "},"
  "{"
  "'name':'minRPM',"
  "'label':'Min RPM (1/min)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1000,'max':20000,"
  "'default':'2900'"
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
  "'default':'49'"
  "},"
  "{"
  "'name':'holdTimeHigh',"
  "'label':'Cut Time - High RPM (ms)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1,'max':1000,"
  "'default':'72'"
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
  "'name':'staggeredCut',"
  "'label':'Staggered Cut Cycles (0-8)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':8,"
  "'default':'2'"
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
  "'name':'category2',"
  "'label':'2-Step RPM Limiter',"
  "'type':"+String(CATEGORY)+""
  "}"
  "{"
  "'name':'limiterFullCut',"
  "'label':'Limiter Full Cut',"
  "'type':"+String(INPUTCHECKBOX)+","
  "'default':'0'"
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
  "'default':'30'"
  "},"
  "{"
  "'name':'category3',"
  "'label':'Misc Options',"
  "'type':"+String(CATEGORY)+""
  "},"
  "{"
  "'name':'pressureInput',"
  "'label':'Pressure Sensor Input',"
  "'type':"+String(INPUTSELECT)+","
  "'options':["
  "{'v':'0','l':'Piezo'},"
  "{'v':'1','l':'ADC 1'},"
  "{'v':'2','l':'ADC 2'}],"
  "'default':'1'"
  "},"
  "{"
  "'name':'buttonInput',"
  "'label':'Button Input',"
  "'type':"+String(INPUTSELECT)+","
  "'options':["
  "{'v':'0','l':'None'},"
  "{'v':'1','l':'ADC 1'},"
  "{'v':'2','l':'ADC 2'}],"
  "'default':'0'"
  "},"
  "{"
  "'name':'buttonMode',"
  "'label':'Button Mode',"
  "'type':"+String(INPUTSELECT)+","
  "'options':["
  "{'v':'0','l':'Disabled'},"
  "{'v':'1','l':'Limiter Hold'},"
  "{'v':'2','l':'Staggered Toggle'}],"
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
  int staggeredCut;

  bool fullCut;
  float wastedSpark;

  bool limiterFullCut;
  int limiterRPM;
  int limiterCut;
  int limiterRetard;

  int pressureInput;
  int buttonInput;
  int buttonMode;
} cfg;

void transferWebconfToStruct(String results)
{
  cfg.retardLow = conf.getInt("retardLow");
  cfg.retardHigh = conf.getInt("retardHigh"); 
  cfg.restore = conf.getInt("restore");

  cfg.minRPM = conf.getInt("minRPM");
  cfg.maxRPM = conf.getInt("maxRPM");

  cfg.holdTimeLow = conf.getInt("holdTimeLow");
  cfg.holdTimeHigh = conf.getInt("holdTimeHigh");

  cfg.deadTime = conf.getInt("deadTime");
  cfg.cutSens = conf.getInt("cutSens");
  cfg.cutHyst = conf.getInt("cutHyst");
  cfg.staggeredCut = conf.getInt("staggeredCut");
  currStag = cfg.staggeredCut;

  cfg.fullCut = conf.getBool("fullCut");
  cfg.wastedSpark = conf.getBool("wastedSpark") ? 360.f : 720.f;

  cfg.limiterFullCut = conf.getBool("limiterFullCut");
  cfg.limiterRPM = conf.getInt("limiterRPM");
  cfg.limiterCut = conf.getInt("limiterCut");
  cfg.limiterRetard = conf.getInt("limiterRetard");

  String pres = conf.getString("pressureInput").c_str();
  String butt = conf.getString("buttonInput").c_str();
  String buttMode = conf.getString("buttonMode").c_str();

  if (pres == "0")
    cfg.pressureInput = 0; // Piezo
  else if (pres == "1")
    cfg.pressureInput = 1; // ADC 1
  else if (pres == "2")
    cfg.pressureInput = 2; // ADC 2

  if (butt == "0")
    cfg.buttonInput = 0; // None
  else if (butt == "1")
    cfg.buttonInput = 1; // ADC 1
  else if (butt == "2")
    cfg.buttonInput = 2; // ADC 2

  if (buttMode == "0")
    cfg.buttonMode = 0; // None
  else if (buttMode == "1")
    cfg.buttonMode = 1; // Limiter Toggle
  else if (buttMode == "2")
    cfg.buttonMode = 2; // Staggered Toggle
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
  <script>
    const valEl1 = document.getElementById('val1');
    const valEl2 = document.getElementById('val2');
    const valEl3 = document.getElementById('val3');
    const valEl4 = document.getElementById('val4');
    const valEl5 = document.getElementById('val5');
    const valEl6 = document.getElementById('val6');
    const ws = new WebSocket(`ws://${location.hostname}:81/`);

    ws.onmessage = function(event) {
      const [val1, val2, val3, val4, val5, val6] = event.data.split(",");
      valEl1.innerText = `Pressure: ${val1}`;
      valEl2.innerText = `RPM: ${val2}`;
      valEl3.innerText = `lastDwell[0]: ${val3}`;
      valEl4.innerText = `targetRetard: ${val4}`;
      valEl5.innerText = `currHoldTime: ${val5}`;
      valEl6.innerText = `currRestore: ${val6}`;
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
    String broadcastString = String(pressureValue) +                "," + String(lastRPM) +       "," + String(lastDwellTime[0]) + ","
                           + String(currRetard) +                "," + String(currHoldTime) +  "," + String(currRestore);

    webSocket.loop();
    webSocket.broadcastTXT(broadcastString);

    delay(200);
  }
}



// Gets executed when (lastDwell + calculated delay) passed after shifting
void IRAM_ATTR on_t0_cut()
{
  GPIO.out_w1ts = (1 << IGBT_PINS[0]);
}
void IRAM_ATTR on_t1_cut()
{
  GPIO.out_w1ts = (1 << IGBT_PINS[1]);
}
void IRAM_ATTR on_t2_cut()
{
  GPIO.out_w1ts = (1 << IGBT_PINS[2]);
}
void IRAM_ATTR on_t3_cut()
{
  GPIO.out_w1ts = (1 << IGBT_PINS[3]);
}

// When ECU changes state of coil (pull low or release)
void coilInterrupt(int ch)
{
  bool bPin = digitalRead(MEASURE_PINS[ch]); // Measurement Pin
  unsigned long lTime = micros();

  // When ECU pulls coil to ground (rising edge on logic signal)
  if (bPin && !lastMeasureState[ch])
  {
    lastRPMdelta[ch] = lTime - dwellBeginTime[ch];

    // Calculates the delay needed to retard the following ignition pulse by currRetard degrees (starting to charge coil right now)
    unsigned long delayForRetard = (currRetard / cfg.wastedSpark) * (lastRPMdelta[ch]);

    // If shift cut is over, smoothly ramp ignition retard back to 0 in currRestore step size
    if (!shiftingTrig && currRetard > 0) {
      currRetard = max(0, currRetard - currRestore);
    }

    dwellBeginTime[ch] = lTime;

    // Either actively retarding ignition or recovering
    if (currRetard > 0)
    {
      if (shiftingTrig && cfg.fullCut && currStag) // TODO: get rid of cfg.fullCut, do normal operation when staggeredCut = 0
      {
        // This lets every cfg.staggeredCut-th ignition pulse go through so you don't collect as much fuel in the exhaust as with a true full cut
        // Should reduce strain on the valvetrain etc. but still do nice pops
        // Problem right now: If full cut would be 7 cycles and you set cfg.staggeredCut to 2 it will do 4 * 2 pulse intervals -> too long
        // but should be good to test if it actually works like expected
        delayForRetard += currStag * lastRPMdelta[ch];

        // This hopefully solves the issue described above.
        // If the expected time to finish the next sub-cut cycle (skip n/staggeredCut ignition cycles) is more than lastRPMdelta (the time between two ignition pulses)
        // it will subtract until it isn't.
        // That way the shift will take a maximum of lastRPMdelta more time than currHoldTime wants it to be.
        // At 7000 RPM 4-cyl wasted spark this would be 8.6ms too much in the absolutele worst case, good enough :)
        // Without this the actual cutTime could have been more than double of what it should have been
        while (currTime + delayForRetard > lastCut + currHoldTime*1000 + lastRPMdelta[ch])
        {
          delayForRetard = max((uint32_t)0, delayForRetard - lastRPMdelta[ch]);
        }
      }

      GPIO.out_w1tc = (1 << IGBT_PINS[ch]); // Close IGBT
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
void IRAM_ATTR onPC3() { coilInterrupt(3); }

void setup()
{
  Serial.begin(57600);

  // LED
  pinMode(GREEN_PIN, OUTPUT);
  digitalWrite(GREEN_PIN, HIGH); // Off default
  pinMode(RED_PIN, OUTPUT);
  digitalWrite(RED_PIN, HIGH); // Off default

  // Hall, Piezo
  int adcPins[3] = {HALL_PINS[0], HALL_PINS[1], PIEZO_PIN};
  for (int i = 0; i < 3; i++) {
    pinMode(adcPins[i], INPUT);
  }

  // Ignition IGBT and measurement (set all 4 channels)
  for (int i = 0; i < 4; i++) {
    pinMode(IGBT_PINS[i], OUTPUT);
    digitalWrite(IGBT_PINS[i], HIGH); // Open IGBT default

    pinMode(MEASURE_PINS[i], INPUT);
  }

  void (*on_cut_funcs[4])() = {on_t0_cut, on_t1_cut, on_t2_cut, on_t3_cut};
  for (int i = 0; i < 4; i++) {
    t_cut[i] = timerBegin(1000000);
    timerAttachInterrupt(t_cut[i], on_cut_funcs[i]);
  }

  // Attach interrupts to MEASURE_PINS[i]
  void (*pcFuncs[4])() = {onPC0, onPC1, onPC2, onPC3};
  for (int i = 0; i < 4; i++) {
    attachInterrupt(digitalPinToInterrupt(MEASURE_PINS[i]), pcFuncs[i], CHANGE);
  }

  conf.setDescription(params);
  conf.readConfig();
  transferWebconfToStruct("");

  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_8_5dBm); // 7 also works (other options: 2, 5, 7, 8.5, 11, 13, 15)
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
  // 1 µs precision (1 MHz)
  currTime = micros();

  // Run every 1 ms (1 kHz)
  if ((currTime - lastCycle) >= 1000)
  {
    // Read ADC sensors every 50 ms (20 Hz)
    if ((currTime - lastRead) >= 50000)
    {
      const int inputs[] = {PIEZO_PIN, HALL_PINS[0], HALL_PINS[1], PIEZO_PIN};

      pressureValue = analogRead(inputs[cfg.pressureInput]);
      if (cfg.pressureInput == 0)
        pressureValue = 4095 - pressureValue; // TODO: Rethink if this makes any sense or if it's obsolete
      
      buttonPressed = cfg.buttonInput ? !digitalRead(inputs[cfg.buttonInput]) : false;

      if (buttonPressed && !lastButtonState)
      {
        switch (cfg.buttonMode)
        {
          case 0: if (limitingRPM)
                  {
                    limitingRPM = false;
                    shiftingTrig = false;
                    cfg.fullCut = conf.getBool("fullCut");
                  }
                  break;

          case 1: limitingRPM = true; break;
          case 2: if (cfg.staggeredCut) currStag = currStag ? 0 : cfg.staggeredCut; break; // TODO: I think this needs two presses on first use to actually toggle
        }
      }

      lastButtonState = buttonPressed;

      if (lastRPMdelta[0] > 0)
      {
        lastRPM = ((cfg.wastedSpark > 700) ? 120000000UL : 60000000UL) / lastRPMdelta[0];
        //lastRPM = (cfg.wastedSpark ? 6000000UL : 12000000UL) / lastRPMdelta[0];
      }

      lastRead = currTime;
    }

    if (pressureValue < (cfg.cutSens - cfg.cutHyst))
      waitHyst = false;

    // 2-step launch control
    if (limitingRPM && cfg.limiterRPM && cfg.limiterCut && cfg.limiterRetard && lastRPM > cfg.limiterRPM && !shiftingTrig)
    {
      shiftingTrig = true;
      waitHyst = true;
      currRetard = cfg.limiterRetard;
      currHoldTime = min((int)((lastRPM - cfg.limiterRPM) * cfg.limiterCut / 100.f), 200); // rpmError * gain and limit to 200 ms
      currRestore = cfg.limiterRetard; // TODO: think of nicer RPM limit algorithm that doesnt need cfg.limiterDiv or similar
      cfg.fullCut = cfg.limiterFullCut;
      lastCut = currTime;
    }

    if (lastRPM < cfg.minRPM)
      waitHyst = true;

    if (pressureValue > cfg.cutSens && lastRPM >= cfg.minRPM && !waitHyst && !shiftingTrig && !limitingRPM && (currTime - lastCut) >= cfg.deadTime*1000)
    {
      shiftingTrig = true;
      currRetard = map(lastRPM, cfg.minRPM, cfg.maxRPM, cfg.retardLow, cfg.retardHigh);
      currHoldTime = map(lastRPM, cfg.minRPM, cfg.maxRPM, cfg.holdTimeLow, cfg.holdTimeHigh);

      currRestore = (int)(currRetard / cfg.restore) + 1;

      waitHyst = true;

      GPIO.out_w1tc = (1 << GREEN_PIN); // On when shifting

      lastCut = currTime;
    }

    if (shiftingTrig && (currTime - lastCut) >= currHoldTime*1000)
    {
      shiftingTrig = false;
      GPIO.out_w1ts = (1 << GREEN_PIN); // LED Off
    }

    lastCycle = currTime;
  }
}