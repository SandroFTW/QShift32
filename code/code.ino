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
#include <esp_now.h>
#include <SPIFFS.h>
#include <WiFi.h>

#include <Arduino.h>

uint8_t broadcastAddress[6];

typedef struct sendData {
  int currGear;
  int currMode;
};

sendData sen;

esp_now_peer_info_t peerInfo;

WebSocketsServer webSocket = WebSocketsServer(81);

// Global variables
volatile unsigned long currTime          = 0;     // current timestamp in 1 µs steps (1 MHz)
volatile unsigned long speedPulses       = 0;     // Pulse count from the wheel speed sensor in the last interval (500 ms / 2 Hz)
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
unsigned long lastSpeedRead       = 0;     // When was the last rear wheel speed measurement (2 Hz)

int currHoldTime                  = 0;     // Interpolated hold time based on current RPM and low/high time
int lastRPM                       = 0;
int lastRPMRate                   = 0;     // RPM change rate (1/min / s)
int lastWheelRPM                  = 0;     // Measured rear wheel speed (1/min)
float lastWheelSpeed              = 0;     // Calculated rear wheel speed (km/h)
int pressureValue                 = 0;     // Piezo/Hall sensor pressure value
int limiterState                  = 0;     // Current state of RPM limiter mode
bool buttonPressed                = false; // Handlebar button state
bool lastButtonState              = false; // Last button state for edge detection
bool waitHyst                     = true;  // Needs to be low before another upshift is allowed
bool decelRetarding               = false; // RPM is dropping fast and ignition retarded

// Gear indicator
//float gearRatios[6] = {2.5, 1.55, 1.15, 0.923, 0, 0};             // Grom gear ratios (Primary: * 3.35) = {8.375, 5.2, 3,85, 3.10, 999, 999}
//float gearRatios[6] = {2.846, 1.947, 1.556, 1.333, 1.190, 1.083}; // XJ6 gear ratios (Primary: * 1.955) = {5.564, 3.806, 3.042, 2.606, 2.326, 2.117}
float measuredRatio = 0.f;
float filteredRatio = 0.f;
float alpha         = 0.f;
int currentGear     = 0; // 1-6, 0 = N

char macAddr[20];

// Timer instances
hw_timer_t *t_cut[4]  = {NULL, NULL, NULL, NULL};

// Pin definitions
const int HALL_PINS[2]    = {10, 11};
const int IGBT_PINS[4]    = {2, 3, 4, 1};
//const int MEASURE_PINS[4] = {9, 14, 7, 8}; // Handwired first revision (xj6)
const int MEASURE_PINS[4] = {9, 7, 8, 12}; // PCB Design Pins (grom, R3b)

const int WHEEL_PIN = 14; // 12 on my grom, 15 on my xj6, 14 in R3b PCB revision
const int PIEZO_PIN = 13;
const int GREEN_PIN = 5;
const int RED_PIN   = 6;

enum lim {
  OFF = 0,
  LAUNCH = 1,
  PIT = 2
};

void macStringToBytes(const char *macStr, uint8_t *macBytes)
{
    for (int i = 0; i < 6; i++)
      macBytes[i] = strtoul(macStr + (i * 3), NULL, 16);
}

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
  "'default':'60'"
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
  "},"
  "{"
  "'name':'limiterAlways',"
  "'label':'Limiter Always Active',"
  "'type':"+String(INPUTCHECKBOX)+","
  "'default':'1'"
  "},"
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
  "'default':'3600'"
  "},"
  "{"
  "'name':'launchRPM',"
  "'label':'Launch Control RPM (1/min)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':20000,"
  "'default':'2500'"
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
  "'name':'limiterDiv',"
  "'label':'Limiter Restore Divisor',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1,'max':20,"
  "'default':'1'"
  "},"
  "{"
  "'name':'limiterMaxSpeed',"
  "'label':'Limiter Max Speed (km/h)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':1000,"
  "'default':'15'"
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
  "'name':'wheelSensor',"
  "'label':'Wheel Speed Sensor',"
  "'type':"+String(INPUTCHECKBOX)+","
  "'default':'0'"
  "},"
  "{"
  "'name':'speedScale',"
  "'label':'Wheel Circumference (mm)',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1,'max':20000,"
  "'default':'1528'"                             // Grom 15/34 (Sprocket) * 1529 (Wheel Dia mm) = 675 (Distance per rear wheel rev)
  "},"
  "{"
  "'name':'sensorPulses',"
  "'label':'Pulses per Wheel Revolution',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1,'max':1000,"
  "'default':'70'"
  "},"
  "{"
  "'name':'category4',"
  "'label':'Gear Indicator',"
  "'type':"+String(CATEGORY)+""
  "},"
  "{"
  "'name':'gearCount',"
  "'label':'Gearbox Gear Count',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':1,'max':6,"
  "'default':'4'"
  "},"
  "{"
  "'name':'gearRatios',"
  "'label':'Gear Ratios (2.00, 1.85, ...)',"
  "'type':"+String(INPUTTEXT)+","
  "'default':'8.375, 5.2, 3,85, 3.10, 999, 999'"     // Grom 3.35 (Primary ratio) * Gearbox Ratio (2.5, 1.55, 1.15, 0.923)
  "},"
  "{"
  "'name':'espNow',"
  "'label':'Enable ESP-NOW Sending',"
  "'type':"+String(INPUTCHECKBOX)+","
  "'default':'0'"
  "},"
  "{"
  "'name':'macAddr',"
  "'label':'Receiver MAC Adress',"
  "'type':"+String(INPUTTEXT)+","
  "'default':'FF:FF:FF:FF:FF:FF'"
  "},"
  "{"
  "'name':'category5',"
  "'label':'Dumb Stuff (Beta)',"
  "'type':"+String(CATEGORY)+""
  "},"
  "{"
  "'name':'enableDecelRetard',"
  "'label':'Enable Decel Retard',"
  "'type':"+String(INPUTCHECKBOX)+","
  "'default':'0'"
  "},"
  "{"
  "'name':'decelThreshold',"
  "'label':'Decel Rate Threshold',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':10000,"
  "'default':'2500'"
  "},"
  "{"
  "'name':'decelRetard',"
  "'label':'Decel Retard',"
  "'type':"+String(INPUTNUMBER)+","
  "'min':0,'max':100,"
  "'default':'20'"
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

  bool limiterAlways;
  bool limiterFullCut;
  int limiterRPM;
  int launchRPM;
  int limiterCut;
  int limiterRetard;
  int limiterDiv;
  int limiterMaxSpeed;

  int pressureInput;
  int buttonInput;
  bool wheelSensor;
  int speedScale;
  int sensorPulses;

  int gearCount;
  float gearRatios[6];
  bool espNow;
  String macAddr;

  bool enableDecelRetard;
  int decelThreshold;
  int decelRetard;
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

  cfg.fullCut = conf.getBool("fullCut");
  cfg.wastedSpark = conf.getBool("wastedSpark") ? 360.f : 720.f;

  cfg.limiterAlways = conf.getBool("limiterAlways");
  cfg.limiterFullCut = conf.getBool("limiterFullCut");
  cfg.limiterRPM = conf.getInt("limiterRPM");
  cfg.launchRPM = conf.getInt("launchRPM");
  cfg.limiterCut = conf.getInt("limiterCut");
  cfg.limiterRetard = conf.getInt("limiterRetard");
  cfg.limiterDiv = conf.getInt("limiterDiv");
  cfg.limiterMaxSpeed = conf.getInt("limiterMaxSpeed");

  String pres = conf.getString("pressureInput").c_str();
  String butt = conf.getString("buttonInput").c_str();

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

  cfg.wheelSensor = conf.getInt("wheelSensor");
  cfg.speedScale = conf.getInt("speedScale");
  cfg.sensorPulses = conf.getInt("sensorPulses");

  cfg.gearCount = conf.getInt("gearCount");

  parseGearRatios(conf.getValue("gearRatios"));

  cfg.espNow = conf.getBool("espNow");
  cfg.macAddr = conf.getString("macAddr").c_str();

  macStringToBytes(cfg.macAddr.c_str(), broadcastAddress);

  cfg.enableDecelRetard = conf.getBool("enableDecelRetard");
  cfg.decelThreshold = conf.getInt("decelThreshold");
  cfg.decelRetard = conf.getInt("decelRetard");
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
  <br>
  <p id="val7">...</p>
  <p id="val8">...</p>
  <p id="val9">...</p>
  <p id="val10">...</p>
  <br>
  <p id="val11">...</p>
  <script>
    const valEl1 = document.getElementById('val1');
    const valEl2 = document.getElementById('val2');
    const valEl3 = document.getElementById('val3');
    const valEl4 = document.getElementById('val4');
    const valEl5 = document.getElementById('val5');
    const valEl6 = document.getElementById('val6');
    const valEl7 = document.getElementById('val7');
    const valEl8 = document.getElementById('val8');
    const valEl9 = document.getElementById('val9');
    const valEl10 = document.getElementById('val10');
    const valEl11 = document.getElementById('val11');
    const ws = new WebSocket(`ws://${location.hostname}:81/`);

    ws.onmessage = function(event) {
      const [val1, val2, val3, val4, val5, val6, val7, val8, val9, val10, val11] = event.data.split(",");
      valEl1.innerText = `Pressure: ${val1}`;
      valEl2.innerText = `RPM: ${val2}`;
      valEl3.innerText = `RPM change: ${val3}`;
      valEl4.innerText = `targetRetard: ${val4}`;
      valEl5.innerText = `currHoldTime: ${val5}`;
      valEl6.innerText = `currRestore: ${val6}`;
      valEl7.innerText = `currentGear: ${val7}`;
      valEl8.innerText = `Gear Ratio: ${val8}`;
      valEl9.innerText = `Speed (km/h): ${val9}`;
      valEl10.innerText = `Limiter Mode: ${val10}`;
      valEl11.innerText = `MAC adress: ${val11}`;
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
    //String sTrig = shiftingTrig ? "true" : "false";
    String cGear = currentGear ? String(currentGear) : "N";
    String lMode = limiterState == OFF ? "OFF" : (limiterState == PIT ? "PIT" : (limiterState == LAUNCH ? "LAUNCH" : "error"));

    String broadcastString = String(pressureValue) +                "," + String(lastRPM) +       "," + String(lastRPMRate) + ","
                              + String(currRetard) +                "," + String(currHoldTime) +  "," + String(currRestore) + ","
                              + cGear +                             "," + String(measuredRatio) + "," + String(lastWheelSpeed) + ","
                              + lMode +                             "," + String(macAddr);

    if (cfg.espNow)
    {
      sen.currGear = currentGear;
      sen.currMode = limiterState;
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&sen, sizeof(sen)); // Send message via ESP-NOW
    }

    webSocket.loop();
    webSocket.broadcastTXT(broadcastString);

    delay(200);
  }
}

// Return the best matching gear for the given gear ratio
int detectGear(float ratio)
{
    if (ratio > 20.0 || ratio < 0.3) return 0; // Assume neutral if ratio is excessively high (rear wheel stationary) or low (clutch pulled at high speed)
    
    int bestGear = 1;
    float minDiff = abs(ratio - cfg.gearRatios[0]);
    
    for (int i = 1; i < cfg.gearCount; i++) {
        float diff = abs(ratio - cfg.gearRatios[i]);
        if (diff < minDiff) {
            minDiff = diff;
            bestGear = i + 1;
        }
    }
    return bestGear;
}

void parseGearRatios(const String& input)
{
  char buf[64];
  input.toCharArray(buf, sizeof(buf));
  char* token = strtok(buf, ",");
  int i = 0;
  while (token && i < 6) {
    cfg.gearRatios[i++] = atof(token);
    token = strtok(nullptr, ",");
  }
}

// // callback when data is sent
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
// {
//   // Serial.print("\r\nLast Packet Send Status:\t");
//   // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
// }



// Gets executed when (lastDwell + calculated delay) passed after shifting
void IRAM_ATTR on_t0_cut()
{
  digitalWrite(IGBT_PINS[0], HIGH);
}
void IRAM_ATTR on_t1_cut()
{
  digitalWrite(IGBT_PINS[1], HIGH);
}
void IRAM_ATTR on_t2_cut()
{
  digitalWrite(IGBT_PINS[2], HIGH);
}
void IRAM_ATTR on_t3_cut()
{
  digitalWrite(IGBT_PINS[3], HIGH);
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

    unsigned long delayForRetard = (currRetard / cfg.wastedSpark) * (lTime - dwellBeginTime[ch]);

    if (!shiftingTrig && currRetard > 0) {
      currRetard = max(0, currRetard - currRestore);
    }

    dwellBeginTime[ch] = lTime;

    // Either actively retarding ignition or recovering
    if (currRetard > 0)
    {
      if (shiftingTrig && cfg.fullCut)
      {
        delayForRetard += ((currHoldTime*1000 / lastRPMdelta[ch]) + 1) * lastRPMdelta[ch];
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
void IRAM_ATTR onPC3() { coilInterrupt(3); }

void IRAM_ATTR onWheelSpeed()
{
  speedPulses++;
}

void setup()
{
  Serial.begin(57600);

  // LED
  pinMode(GREEN_PIN, OUTPUT);
  digitalWrite(GREEN_PIN, HIGH); // Off default
  pinMode(RED_PIN, OUTPUT);
  digitalWrite(RED_PIN, HIGH); // Off default

  // Hall, Piezo
  int adcPins[4] = {HALL_PINS[0], HALL_PINS[1], PIEZO_PIN, WHEEL_PIN};
  for (int i = 0; i < 4; i++) {
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

  // Rear wheel speed sensing
  attachInterrupt(digitalPinToInterrupt(WHEEL_PIN), onWheelSpeed, RISING);

  conf.setDescription(params);
  conf.readConfig();
  transferWebconfToStruct("");

  WiFi.mode(WIFI_AP_STA); // WIFI_AP

  esp_now_init(); // Init ESP-NOW
  //esp_now_register_send_cb(OnDataSent); // Hopefully the send callback is not required

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  esp_now_add_peer(&peerInfo); // Add peer

  WiFi.setTxPower(WIFI_POWER_8_5dBm); // 7 also works (other options: 2, 5, 7, 8.5, 11, 13, 15)
  WiFi.softAP(conf.getApName(), conf.values[0].c_str());

  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    sprintf(macAddr, "%02x:%02x:%02x:%02x:%02x:%02x\n", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  }

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
        pressureValue = 4095 - pressureValue;
      
      buttonPressed = cfg.buttonInput ? !digitalRead(inputs[cfg.buttonInput]) : false;

      if (buttonPressed && !lastButtonState)
      {
        switch (limiterState)
        {
          case PIT:    limiterState = OFF; break;
          case LAUNCH: limiterState = OFF; break;

          case OFF:
            if (lastWheelSpeed < cfg.limiterMaxSpeed && cfg.wheelSensor)
              limiterState = LAUNCH;
            else
              limiterState = PIT;
            break;
        }
      }

      if (cfg.limiterAlways)
        limiterState = PIT;                                    // TODO: Possible conflict with decelRetarding

      lastButtonState = buttonPressed;

      int backupRPM = lastRPM;
      if (lastRPMdelta[0] > 0)
      {
        lastRPM = ((cfg.wastedSpark > 700) ? 120000000UL : 60000000UL) / lastRPMdelta[0];
        //lastRPM = (cfg.wastedSpark ? 6000000UL : 12000000UL) / lastRPMdelta[0];
        //lastRPMRate = (lastRPM - backupRPM);
        lastRPMRate = 0.1 * 20*(lastRPM - backupRPM) + 0.9 * lastRPMRate;
      }

      lastRead = currTime;
    }

    // Read wheel speed every 500 ms (2 Hz)
    if (cfg.wheelSensor && (currTime - lastSpeedRead) >= 500000)
    {
      lastWheelRPM = (int)(speedPulses * (120 / (float)cfg.sensorPulses));
      speedPulses = 0;

      lastWheelSpeed = (lastWheelRPM * cfg.speedScale) / 60000.f;

      measuredRatio = (lastWheelRPM > 0) ? (lastRPM / (float)lastWheelRPM) : 999.9f; // Calculate gear ratio from engine RPM and rear wheel RPM
      filteredRatio = (alpha * measuredRatio) + ((1 - alpha) * filteredRatio); // Exponential moving average filtering

      currentGear = detectGear(measuredRatio);

      lastSpeedRead = currTime;
    }

    if (pressureValue < (cfg.cutSens - cfg.cutHyst))
      waitHyst = false;

    // 2-step launch control
    if ((cfg.launchRPM || cfg.limiterRPM) && cfg.limiterCut && cfg.limiterRetard)
    {
      switch (limiterState)
      {
        case LAUNCH:
          if (lastWheelSpeed > cfg.limiterMaxSpeed)
            limiterState = OFF;

          if (cfg.launchRPM && lastRPM > cfg.launchRPM && !shiftingTrig)
          {
            shiftingTrig = true;
            limitingRPM = true;
            waitHyst = true;
            currRetard = cfg.limiterRetard;
            currHoldTime = min((int)((lastRPM - cfg.launchRPM) * cfg.limiterCut / 100.f), 200); // rpmError * gain and limit to 200 ms
            currRestore = (int)(cfg.limiterRetard / cfg.limiterDiv);
            cfg.fullCut = cfg.limiterFullCut;
            lastCut = currTime;
          }
          break;

        case PIT:
          if (cfg.limiterRPM && lastRPM > cfg.limiterRPM && !shiftingTrig)
          {
            shiftingTrig = true;
            limitingRPM = true;
            waitHyst = true;
            currRetard = cfg.limiterRetard;
            currHoldTime = min((int)((lastRPM - cfg.limiterRPM) * cfg.limiterCut / 100.f), 200); // rpmError * gain and limit to 200 ms
            currRestore = (int)(cfg.limiterRetard / cfg.limiterDiv);
            cfg.fullCut = cfg.limiterFullCut;
            lastCut = currTime;
          }
          break;

        case OFF:
          if (limitingRPM && !decelRetarding)
          {
            limitingRPM = false;
            shiftingTrig = false;
            cfg.fullCut = conf.getBool("fullCut");
          }
          break;
      }
    }

    if (cfg.enableDecelRetard) // If it works like this, this check can be moved back to the long if condition
    {
      if (lastRPM > cfg.minRPM && (-1 * lastRPMRate) > cfg.decelThreshold && limiterState == OFF && !limitingRPM)
      {
          decelRetarding = true;
          shiftingTrig = true;
          waitHyst = true;
          currHoldTime = 100;
          currRetard = cfg.decelRetard;
          currRestore = 999;
          cfg.fullCut = false;
      }
      else
      {
        if (decelRetarding)
        {
          decelRetarding = false;
          shiftingTrig = false;
          cfg.fullCut = conf.getBool("fullCut");
        }
      }
    }

    if (lastRPM < cfg.minRPM)
      waitHyst = true;

    if (pressureValue > cfg.cutSens && lastRPM >= cfg.minRPM && !waitHyst && !shiftingTrig && !limitingRPM && !decelRetarding && (currTime - lastCut) >= cfg.deadTime*1000)
    {
      shiftingTrig = true;
      currRetard = map(lastRPM, cfg.minRPM, cfg.maxRPM, cfg.retardLow, cfg.retardHigh);
      currHoldTime = map(lastRPM, cfg.minRPM, cfg.maxRPM, cfg.holdTimeLow, cfg.holdTimeHigh);

      currRestore = (int)(currRetard / cfg.restore) + 1;

      waitHyst = true;

      digitalWrite(GREEN_PIN, LOW); // On when shifting

      lastCut = currTime;
    }

    if (shiftingTrig && (currTime - lastCut) >= currHoldTime*1000)
    {
      shiftingTrig = false;
      digitalWrite(GREEN_PIN, HIGH); // Off
    }

    lastCycle = currTime;
  }
}