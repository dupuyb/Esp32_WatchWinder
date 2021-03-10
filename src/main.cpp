#include "driver/rtc_io.h" // brownout detector
// Stepper
#include <AccelStepper.h>
// Frame wifi
#include "FrameWeb.h"
FrameWeb frame;

// WWM config file
#include "ConfigWWM.h"
ConfigWWM Confwwm;
// http client
#include <HTTPClient.h>
// Accelerometer
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <PubSubClient.h>
#include "CalibrationWWM.h"
#include "DspEpaper.h"
#include "Trig.h"
#include "WatchDog.h"

// mqtt 
String MqttIn;
String MqttOut;
uint8_t mqttCrc8 = 0;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Power supply-survey & wifi
WatchDog wdogPwr;
int8_t wifiLost = 0;
#define pinAd 39 // ADC1_3 day/night detector
uint16_t lux;
#define pinAo 36 // ADC1_0 A 
#define pwrP5V 32 // Primaire 5V
#define pwrS5V 4 // Relay power-supply is on at 1
#define pinSp 5  // Hall cadran position detector
#define pinGl 21 // Presence Glass
Trig glassOk(true);
#define pinT8 33 // Touch sensor Not used now.
 //#define pinT9 32 pwrP5V inside of 34 input
uint16_t vdayNi = 0xFFF; // Set at max
uint16_t vddUsb;
#define SCAL5V 0.001221
uint16_t lowLimit = 0xF5B; // USB is connected 4.8V
Trig dayMode(true);
int dayModeCnt=0;

const char *version = "1.1.3";
const char *HOSTNAME = "WatchWinderMax";
int wifiSt = -1;

// Time facilities
struct tm timeinfo;                  // time struct

// for SPI pin definitions see e.g. \Documents\Arduino\hardware\espressif\esp32\variants\lolin32\pins_arduino.h
// BUSY -> GPIO4, RST -> GPIO2, DC -> GPIO0, CS -> GPIO15, CLK -> GPIO14, DIN -> GPIO13, GND -> GND, 3.3V -> 3.3V
#define PIN_SPI_SCK 13
#define PIN_SPI_DIN 14
#define PIN_SPI_CS 15
#define PIN_SPI_BUSY 25
#define PIN_SPI_RST 26
#define PIN_SPI_DC 27
SPIClass spi(2);                                         // mapped on PIN_SPI_SCK PIN_SPI_DIN, PIN_SPI_CS spi_in is not used
GxIO_Class io(spi, PIN_SPI_CS, PIN_SPI_DC, PIN_SPI_RST); // arbitrary selection of 17, 16
GxEPD_Class display(io, PIN_SPI_RST, PIN_SPI_BUSY);      // arbitrary selection of (16), 4

// Stepper Gear reduction	1/64 (see note)  64*64 = 4096 per turn
#define stepsPerRevolution 8
#define pinU1 16 // Selenoid
#define pinU2 17 // Selenoid
#define pinU3 18 // Selenoid
#define pinU4 19 // Selenoid
AccelStepper stepper(stepsPerRevolution, pinU1, pinU3, pinU2, pinU4);

// Accelerometer LIS3DH 3-axis accelerometer
#define pinSDA 23
#define pinSCL 22
Adafruit_LIS3DH *lis3dh;
// Adjust this number for the sensitivity of the 'click' force
// this strongly depend on the range! for 16G, try 5-10
// for 8G, try 10-20. for 4G try 20-40. for 2G try 40-80
uint8_t lis3dhClick = 0;
bool lis3dhPresent;

// Timer
#define LED
#ifdef LED
#define EspLedBlue 2
#endif
long previousMillis = 0;

// Stepper Calibration 
CalibrationWWM calibStp(&stepper);

// Frame option
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {}
void configModeCallback(WiFiManager *myWiFiManager) {}
void saveConfigCallback() {}
// Declaration
void mqttPostionMsg(long pos, String src);
// Display
String getPRoperty(int);
DspEpaper eDsp(&display, &getPRoperty, &timeinfo);
// Task
static int app_cpu = 0; // Updated by setup() 
QueueHandle_t qDsp = xQueueCreate(5, sizeof(qmsgstuct));
TaskHandle_t epaperCxHandle = NULL;
TaskHandle_t stepperCxHandle = NULL;
TaskHandle_t wifiCxHandle = NULL;

#define DSP_TASK_PRIORITY 1
#define WIFI_TASK_PRIORITY 7
#define MAIN_TASK_PRIORITY  10
#define STEPPER_TASK_PRIORITY 12 // Low priority numbers denote low priority tasks

// Debug
int8_t cmd;
// LOGGER update 120 chars Max
#define LOG(format, ...) { \
  if(cmd!=' ') { \
    char temp[121];\
    snprintf(temp, 120, format, __VA_ARGS__); \
    Serial.println(temp); \
  } \
}  

#define BOOL(val) ((val)?("On"):("Off"))

// -------- WWM Web transformation into Get Set functions ------------- 
#include "wwmconfig.h"

//#define tI2CTST
#ifdef I2CTST
// I2C scanning address
void scanI2C() {
  Serial.printf("Start Scanning I2C Addresses pinSDA:%d pinSCL:%d\n\r", pinSDA, pinSCL);
  uint8_t cnt = 0;
  for (uint8_t i = 0; i < 128; i++) {
    Wire.beginTransmission(i);
    uint8_t ec = Wire.endTransmission(true);
    if (ec == 0) {
      if (i < 16)
        Serial.print('0');
      Serial.print(i, HEX);
      cnt++;
    }  else
      Serial.print("..");
    Serial.print(' ');
    if ((i & 0x0f) == 0x0f)
      Serial.println("");
  }
  Serial.print("Scan Completed, ");
  Serial.print(cnt);
  Serial.println(" I2C Devices found.");
}
#endif

String getPRoperty(int prop) {
  String ret;
  char temp[20];
  switch (prop) {
      case ICON: {
        String ret = "0";
        if (lis3dhClick != 0) ret = "1";
        else if (calibStp.turnCalRun || calibStp.autoCalRun) ret = "2";
        else if (calibStp.stepper->isRunning()) ret = "3"; 
        else if (wifiSt != WL_CONNECTED) ret = "4";
        else if (!glassOk.get()) ret = "5";
        if (wdogPwr.isArmed()) ret = "6"; // Priority
        return ret;
      }
      break;
    case WIFI:
      if (wifiSt == WL_CONNECTED) return String("OK");
      return String("BAD");
    break;
    case HHMM:
      if (timeinfo.tm_year > 100) snprintf(temp, 10, "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
      else strcpy(temp, "H:M");
      return String(temp);
    break;
    case DDMM:
      if (timeinfo.tm_year > 100) snprintf(temp, 20, "%02d/%02d/%04d", timeinfo.tm_mday, (timeinfo.tm_mon + 1), (1900 + timeinfo.tm_year));
      else  strcpy(temp, "D/M");
      return String(temp);     
    case DMY:
      if (timeinfo.tm_year > 100) snprintf(temp, 20, "%02d-%02d-%04d", timeinfo.tm_mday, (timeinfo.tm_mon + 1), (1900 + timeinfo.tm_year));
      else  strcpy(temp, "D-M-Y");
      return String(temp);
    break;
    case MAC:
      uint8_t baseMac[6];
      esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
      snprintf(temp, 20, "%02X:%02X:%02X:%02X:%02X:%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
      return String(temp);
    break;
    case VER: return String(version);  break;
    case SPOS: { snprintf(temp, 20, "%ld", stepper.currentPosition());return String(temp);} break;
    case SSPE: { snprintf(temp, 20, "%.1f", stepper.speed());return String(temp);} break;
    case SISR: { snprintf(temp, 20, "%s", ((stepper.isRunning())?("ON"):("OFF"))); return String(temp);} break;
    case SCAL: { snprintf(temp, 20, "%ld %ld", calibStp.ccwl, calibStp.ccwh); return String (temp); } break;
  }
  return String("Bad Prop:") + String (prop);
}

// -------------------------------------------------------
void explorerFS(String &ret, fs::FS &fs, const char *dirname, uint8_t levels) {
  File root = fs.open(dirname);
  if (!root)   return;
  if (!root.isDirectory())   return;
  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      if (levels)
        explorerFS(ret, fs, file.name(), levels - 1);
    }  else {
      String strf = (file.name());
      ret += strf;
      ret += "\n\r";
    }
    file = root.openNextFile();
  }
  return;
}

//-------------------------------------------------
HTTPClient http;  
void httpGetFile (String name) {
  //http.setTimeout(60);
  http.begin( Confwwm.config.ImageHttpEa + name);
  http.addHeader("Content-Type", "text/plain");                       //Specify content-type header
  int httpResponseCode = http.POST("POSTING from WatchWinderMax");   //Send the actual POST request
  if(httpResponseCode>0) {
    String response = http.getString();                               //Get the response to the request
      if (httpResponseCode==200){   
      File file = SPIFFS.open(name, "w");
      if (!file) {
        LOG("%s -Can't load image:%s", getPRoperty(HHMM).c_str(), name.c_str());
      } else {
        file.print(response),
        file.close();
      }
    }
  } else {
    LOG("%s -Error sending POST: %d for:%s", getPRoperty(HHMM).c_str(), httpResponseCode, name.c_str());
  }
  http.end();  //Free resources
}

void getWebTask() {
  char str[12];
  LOG("%s +Start taskGetImageWeb",getPRoperty(HHMM).c_str());
  for (int i=0; i<10; i++) {
    snprintf(str, 12, "/web%d.jpg", i); 
    httpGetFile(String(str));
    yield();
    vTaskDelay(35);
  }
  LOG("%s +Set UTP time",getPRoperty(HHMM).c_str());
  configTime(Confwwm.config.gmtOffset_sec, Confwwm.config.daylightOffset_sec, Confwwm.config.ntpServer); //init and get the time
  LOG("%s -Finish taskGetImageWeb",getPRoperty(HHMM).c_str());
}  

void IRAM_ATTR wifiTask(void *pvParameter) {
  if (WiFi.status() != WL_CONNECTED) {
    LOG("%s +wifiTask started",getPRoperty(HHMM).c_str());
    frame.externalHtmlTools="Specific home page is visible at :<a class='button' href='/wwm'>W.W.M Page</a>";
    frame.setup(HOSTNAME); // SPIFS, WIFI, OTA, Server stating...
    configTime(Confwwm.config.gmtOffset_sec, Confwwm.config.daylightOffset_sec, Confwwm.config.ntpServer); //init and get the time
    LOG("%s -wifiTask ok delte task",getPRoperty(HHMM).c_str());
  }
  vTaskDelete( NULL ); // auto destroy
}

// Display Task waitting message in qDsp queue
void IRAM_ATTR epaperTask(void *pvParameter) {
  LOG("%s +Start epaperTask & manager",getPRoperty(HHMM).c_str());
  while (1) { 
    qmsgstuct dmv;
    xQueueReceive (qDsp, &dmv, portMAX_DELAY);
    if (dmv.evt==dsp) eDsp.mainLoop(dmv);
    if (dmv.evt==web) getWebTask();
    vTaskDelay( pdMS_TO_TICKS( 100 ) );
  }
}

// Increment or Decrement step
void IRAM_ATTR stepperTask(void *pvParameter) {
  LOG("%s +Start stepperTask",getPRoperty(HHMM).c_str());
  while (1) {
    if (stepper.run()) {
      stepper.enableOutputs();
      vTaskDelay(1);
     } else {
       stepper.disableOutputs();
       vTaskDelay(pdMS_TO_TICKS( 100 ) );
     }
  }
}

// Interrupt if Hall sensor A3114 detects the cadran magnet (Call in change mode)
// Moving (-->)   ccw ---ccwl____>>_____ccwh-----   Here value of ccwl < ccwh
// Moving (<--)   acw ---acwh____<<_____acwl----    Here value of acwl > acwh
void IRAM_ATTR positionCHG() {
  int val = digitalRead(pinSp);
  long cp = stepper.currentPosition();
  long tp = stepper.targetPosition();
  calibStp.dirCcw = tp > cp;
  if (val==HIGH) { 
    if (calibStp.dirCcw) calibStp.ccwh = cp;
    else calibStp.acwh = cp;
    calibStp.isAtNoth = false;
  } else { // We are outside of the detector --> reset edge high to prevent turn-around 
    if (calibStp.dirCcw) { calibStp.ccwl = cp; calibStp.ccwh = NV; }
    else { calibStp.acwl = cp; calibStp.acwh = NV; }
    calibStp.isAtNoth = true;
  }
}

// adapter topic message ---------------------------------
void mqttPublish(String src, String jkey, String jval){
  String msg;
  DynamicJsonDocument json(80);
  json["From"] = src;
  json[jkey] = jval;
  serializeJson(json, msg);
  LOG("%s +MqttPublish Topic[%s] msg[%s]\n\r",getPRoperty(HHMM).c_str(), MqttOut.c_str(), msg.c_str());
  mqttClient.publish(MqttOut.c_str(), msg.c_str());
}

// {"BadDir":"500","BroAdr":"192.168.1.117","BroPor":"1883","BroPwd":"","BroUse":"","DayOff":"3600","GmtOff":"-21600"}
void mqttPublishAll(){ // size limeted at MQTT_MAX_PACKET_SIZE+128 byte
  for (int idx=0; idx<NBRITEMINDICO; idx++) {
    String msg;
    DynamicJsonDocument json(1000);
    if (dico[idx].get_ptr != NULL) {
      String k = String(dico[idx].key);
      k.replace("%%", "");
      json[k]=(*dico[idx].get_ptr)(false);
      serializeJson(json, msg);
      mqttClient.publish(MqttOut.c_str(), msg.c_str());
    }
  } 
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  LOG("%s +MQTT callback [%s]", getPRoperty(HHMM).c_str(), topic); 
  std::unique_ptr<char[]> buf(new char[length]);
  for (int i=0;i<length; i++) {
    buf.get()[i]=(char)payload[i];
  }
  String msg(buf.get()); 
  DynamicJsonDocument json(128);
  if (deserializeJson(json, buf.get())){ 
    return;
  }
  JsonObject documentRoot = json.as<JsonObject>();
  bool empty = true;
  for (JsonPair keyValue : documentRoot) {
    String val = json[keyValue.key()];
    String key = "%%"+String(keyValue.key().c_str())+"%%";
    LOG("%s +MQTT Callback [%s]:[%s]\n\r",getPRoperty(HHMM).c_str(), key.c_str(), val.c_str() );
    for (int idx=0; idx<NBRITEMINDICO; idx++) {
       if (key==dico[idx].key && dico[idx].set_ptr != NULL) {
        (*dico[idx].set_ptr)(val);
        empty = false;
        break;
      }
    } 
  }
  //  Print
  String output;
  serializeJson(json, output);
  LOG("%s", output.c_str());
  if (empty){
    mqttPublishAll();
  } 
}

bool mqttReconnect() {
  // Loop until we're reconnected
  if (!mqttClient.connected()) {
    String clientId = frame.config.HostName;
    clientId += String(random(0xffff), HEX);
    LOG("%s +mqttReconnect.",getPRoperty(HHMM).c_str());
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str())) {
      // Once connected, publish an announcement... and resubscribe 
      mqttPublish("Mqtt", "Connection", "true");
      mqttClient.subscribe(MqttIn.c_str());
    } else {
      return false;
    }
  }
  return true;
}

void setupDisplay(){
  // Adapte to GDEW0213I5F 212x104, 2.13inch flexible E-Ink display on ePaperESP32 Driver Board
  spi.begin(PIN_SPI_SCK, -1, PIN_SPI_DIN, PIN_SPI_CS);
  display.init();
  display.setRotation(1);
}

void setup(void) {
  // Set Serial ---------------------------------------
  Serial.begin(115200);
  if ( esp_reset_reason()==ESP_RST_SW ) cmd='d';
  if (cmd=='d') Serial.println("Start -[d]-");
  else Serial.println("Start...");
#ifdef LED
  pinMode(EspLedBlue, OUTPUT);     // Led is BLUE at statup
  digitalWrite(EspLedBlue, HIGH);  // After 5 seconds blinking indicate WiFI ids OK
#endif
  // Arm power ----------------------------------------
  pinMode(pwrP5V, OUTPUT);
  pinMode(pwrS5V, OUTPUT);
  // MUST BE IMPROVED
  digitalWrite(pwrP5V, LOW);    // initial state power ON
  digitalWrite(pwrS5V, LOW);    // initial state power ON
  // Set Power-supply survey & position capture
  pinMode(pinAo, INPUT);
  pinMode(pinSp, INPUT);
  pinMode(pinGl, INPUT);
  pinMode(pinAd, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinSp), positionCHG, CHANGE);
  // Load configuration
  Confwwm.loadConfigurationJson();
  // TEST A FAIRE
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  // Set Task priority --------------------------------
  TaskHandle_t h = xTaskGetCurrentTaskHandle();   
  app_cpu = xPortGetCoreID(); // Quelle UC ?   
  vTaskPrioritySet(h,MAIN_TASK_PRIORITY);
  // setup the display --------------------------------
  setupDisplay();
  // first message welcome message
  xTaskCreate( &epaperTask, "epaperTask", 8096, NULL, DSP_TASK_PRIORITY, nullptr);   
  qmsgstuct dmv {start, day, 0, dsp};
  xQueueSend(qDsp, &dmv, 0);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  // WiFi Task ----------------------------------------
  if (app_cpu) disableCore0WDT();
  else disableCore1WDT();
  xTaskCreate(&wifiTask, "wifiTask", 4096,  NULL, WIFI_TASK_PRIORITY,  &wifiCxHandle);
  // Init Stepper And StepperTask (Speed, MaxSpeed Accelea=ration) ------------
  calibStp.init(100.0, 800.0, 400.0);
  xTaskCreate( &stepperTask, "stepperTask", 2048,  NULL, STEPPER_TASK_PRIORITY, &stepperCxHandle);
  // Wire i2c for Lis30H --------------------------------
  Wire.begin(pinSDA, pinSCL);
  lis3dh = new Adafruit_LIS3DH(&Wire);
#ifdef I2CTST
  scanI2C();
#endif
  if (!lis3dh->begin(0x19)){ // change this to 0x18 for alternative i2c address
    lis3dhPresent = false;
    LOG("/!\\ %s Error LIS3DH not found!", getPRoperty(HHMM).c_str());
  } else {
    lis3dhPresent = true;
  }
  lis3dh->setRange(LIS3DH_RANGE_2_G); // 2, 4, 8 or 16 G!
  // Adjust threshold, higher numbers are less sensitive
  lis3dh->setClick(2, Confwwm.config.clickThreshold);
  // Append /wwm access html 
  frame.server.on("/wwm", [](){
    frame.server.send(HTTP_CODE_OK, "text/html", sentHtmlWwm());
  });
  sleep(2);
  yield();
}

void mqttPostionMsg(long pos, String src){
  if (!src.isEmpty() && mqttClient.connected()) {
    mqttPublish(src, "Cmd", String(pos));
  }
  stepper.moveTo(pos);
}

void tocAction (int t){
  LOG("%s +tocAction:%d",getPRoperty(HHMM).c_str(),t);
  long absoluP = 0;
  if (calibStp.isAtNoth) {
    if (stepper.currentPosition()>Confwwm.config.oneTurnInStep-50) absoluP=0;
    else absoluP=Confwwm.config.oneTurnInStep * t;
  }
  mqttPostionMsg(absoluP, "Knock");
} 

void clockAction (int h, int m){
  LOG("%s +clockAction hour:%d",getPRoperty(HHMM).c_str(),h);
  long absoluP = 0;
  if (h>12) h=h-12; // too long for 22h 23h....
  if ( m==0)  
    absoluP = Confwwm.config.oneTurnInStep * h;
  mqttPostionMsg(absoluP, "Clock");
} 

void clockAction(int h, int m, int s){
  if (m==0) LOG("%s +clockAction minute (1/h)",getPRoperty(HHMM).c_str());
  float oneminute = (float)Confwwm.config.oneTurnInStep/60.0;
  float ntrun = oneminute * (float)m;
  if (m==30 ) {
    LOG("%s +clockAction minute (-1.5/h)",getPRoperty(HHMM).c_str());
    ntrun = -ntrun; // -1 turn at xxH30
  }
  long absoluP  = (long)ceil(ntrun);
  mqttPostionMsg(absoluP, "");
}

void printHelp() {
  Serial.println("------- Help command available -----------------|");
  Serial.println("  i     Info system (heap,ip, detectors...)     |");
  Serial.println("  d     Debug RT (log message)                  |");
  Serial.println("  r     Reboot system                           |");
  Serial.println("  z     Force zero if Meanposition changed      |");
  Serial.println("  s/S   Screen information / Set low power      |");
  Serial.println("  t/T   Time/date information / Test Stepper    |");
  Serial.println("  c/C   Calibate stepper Postition/Turn         |");
  Serial.println("  -/0/1 Move stepper -:-1Turn 0:Goto0 1:+1Truns |");
  Serial.println("  </>   Move stepper 1/10 turn <-- or -->       |");
  Serial.println("  #     Reset Edges limit motor                 |"); 
  Serial.println("  m     Motor status                            |"); 
  Serial.println("  p/P   Power 5V_chargeur Off/On                |"); 
  Serial.println("  q/Q   Power 5V_backup   Off/On                |"); 
  Serial.println("  g     Get http web0.jpg                       |"); 
  Serial.println("  w     Write setup display test                |"); 
  Serial.println("------------------------------------------------|"); 
}

void serialInput() {
  while (Serial.available() > 0) {
    uint8_t c = (uint8_t)Serial.read();
    if (c != 13 && c != 10 ) {
      cmd = c;
    } else {
      if (c==13) {
        if (cmd=='h') { printHelp(); }
        else if (cmd=='i') { 
          Serial.println("-[i]-- info system ---");
          Serial.printf("-HeapSize:%u bytes\n\r-MacAddr :%s\n\r-IpAddr  :%s\n\r",ESP.getFreeHeap(),getPRoperty(MAC).c_str(),WiFi.localIP().toString().c_str());
          Serial.println("------sensors---------");
          Serial.printf("-Dome :%s\n\r-Light:%d LimitDay:<%d\n\r",((glassOk.get()==1)?("detected"):("undetected")),lux,Confwwm.config.dayTwilight),
          Serial.printf("-Knock:%d Toctoc isAtNorth:%d \n\r",lis3dhClick, calibStp.isAtNoth);
          Serial.printf("-PowerUSB :%1.1fV lowPwr:<%1.1fV\n\r",(vddUsb*SCAL5V),(lowLimit*SCAL5V) );
          Serial.printf("-PowP:%s PowB:%s\n\r", BOOL(digitalRead(pwrP5V)), BOOL(digitalRead(pwrS5V))); 
          cmd=' '; 
        }
        else if (cmd=='d') { Serial.println("-[d]-- Debug ---\n\r Other cmd to stop"); cmd='d'; }
        else if (cmd=='r') { Serial.println("-[r]-- Reboot ---"); ESP.restart(); }
        else if (cmd=='s') { Serial.printf("-[s]-- Screen ---\n\r%s\n\r",eDsp.toString().c_str()); cmd='d';}
        else if (cmd=='S') { Serial.println("-[S]-- Stop ---"); lowLimit=0xf001; cmd='d';}
        else if (cmd=='t') { Serial.printf("-[t]-- Time ---\n\rTime:%s  Date:%s day:%d\n\r", getPRoperty(HHMM).c_str(), getPRoperty(DMY).c_str(), timeinfo.tm_wday); cmd='d';}
        else if (cmd=='T') { Serial.println("-[T]-- Test stepper ---"); calibStp.testStepper=!calibStp.testStepper; cmd='d';}
        else if (cmd=='C') { Serial.println("-[C]-- Calibration ---"); Serial.println("Mode calibration turn active."); calibStp.turnCalRun=1; cmd='d'; }
        else if (cmd=='c') { Serial.println("-[c]-- Calibration ---"); Serial.println("Mode calibration position active."); calibStp.autoCalRun=1; cmd='d'; }
        else if (cmd=='>') { Serial.printf("-[>]-- Move CCW ---\n\rStepper is moving relative (%ld) step\n\r", (Confwwm.config.oneTurnInStep/10));  stepper.move((Confwwm.config.oneTurnInStep/10)); cmd='d';}
        else if (cmd=='<') { Serial.printf("-[<]-- Move ACW ---\n\rStepper is moving relative (%ld) step\n\r", (Confwwm.config.oneTurnInStep/-10));  stepper.move((Confwwm.config.oneTurnInStep/-10)); cmd='d';}
        else if (cmd=='#') { Serial.println("-[#]-- Reset Edge ---"); calibStp.resetEdge(); cmd='d';}
        else if (cmd=='-') { Serial.printf("-[-]-- Move ---\n\rStepper is moving relatif to %ld setps\n\r", (Confwwm.config.oneTurnInStep*-1)); stepper.move((Confwwm.config.oneTurnInStep*-1)); cmd='d';}
        else if (cmd=='0') { Serial.println("-[0]-- Move ---\n\rStepper is moving absolute to 0 step\n\r");stepper.moveTo(0); cmd='d';}
        else if (cmd=='1') { Serial.printf("-[2]-- Move ---\n\rStepper is moving relatif to %ld steps\n\r", (Confwwm.config.oneTurnInStep*1)); stepper.move(Confwwm.config.oneTurnInStep*1); cmd='d'; }
        else if (cmd=='m') { 
          Serial.printf("-[m]-- Motor ---\n\rCurrent postion:%ld  Speed:%f isRun:%s isNorth:%d \n\r",stepper.currentPosition(), stepper.speed() , ((stepper.isRunning()==1)?("Yes"):("No")), calibStp.isAtNoth);  cmd=' ';
          Serial.printf("1Turn:%ld target:%ld ccwl:%ld ccwh:%ld acwl:%ld acwh:%ld checkMean:%ld \n\r", Confwwm.config.oneTurnInStep, stepper.targetPosition(), calibStp.ccwl, calibStp.ccwh, calibStp.acwl, calibStp.acwh, calibStp.checkMean());
        }
        else if (cmd=='z') { Serial.printf("-[z]-- Move ---\n\rStepper move to CheckMean:%ld\n\r",calibStp.checkMean());  calibStp.forceZero(calibStp.checkMean()); cmd='d';}
        else if (cmd=='g') { Serial.printf("-[g]-- Http ---\n\rGet:%s/web0.jpg image on server\n\r", Confwwm.config.ImageHttpEa); qmsgstuct dmv {normal, param, 0, web}; xQueueSend(qDsp, &dmv, 0); cmd='d';}
        else if (cmd=='p') { Serial.println("-[p]-- Power ---\n\rChargeur switch OFF"); digitalWrite(pwrP5V, LOW); cmd='d';}
        else if (cmd=='P') { Serial.println("-[P]-- Power ---\n\rChargeur switch ON"); digitalWrite(pwrP5V, HIGH); cmd='d';}
        else if (cmd=='q') { Serial.println("-[q]-- Power ---\n\rBackup switch OFF"); digitalWrite(pwrS5V, LOW); cmd='d';}
        else if (cmd=='Q') { Serial.println("-[Q]-- Power ---\n\rBackup switch ON"); digitalWrite(pwrS5V, HIGH); cmd='d';}
        else if (cmd=='w') { Serial.println("-[w]-- Epaper ---\n\rReboot display"); setupDisplay(); cmd=' ';}
        else { Serial.printf("-[%c]-- Command not found. Serial is stopped\n\r",cmd); }
      }
    }
  }
}

//-------------  Main task  ------------------------------------
void loop(void) {
  // brownout detector
  vddUsb = analogRead(pinAo); // Detect if USB power is down
  if ( vddUsb < lowLimit) {
    if ( wdogPwr.isFree() ) {
      LOG("%s -Broutput detected.",getPRoperty(HHMM).c_str());
      digitalWrite(pwrS5V, HIGH);
      long rest = stepper.currentPosition() % Confwwm.config.oneTurnInStep;
      wdogPwr.set( 60000 ); // 60sec/turn max to go at rest position after that cut off
      stepper.move(-rest); // sent dial at correct position.
      mqttPublish("Mqtt", "Connection", "false");
    } 
  }

  // Stop in case of every thing is ok -> Lipo set to switch off
  if ( (eDsp.getScreen()==STOP && calibStp.isAtNoth ) || wdogPwr.isFinished() ) {
     if (digitalRead(pwrS5V)==HIGH) LOG("%s -Switch power OFF.",getPRoperty(HHMM).c_str());
     digitalWrite(pwrS5V, LOW);  // Switch main power Off
    // return;
  }

  // Glass Presente
  if ( digitalRead(pinGl)==HIGH) glassOk.set(false);
  else glassOk.set(true);

  // Call wifi loop
  frame.loop();

  // Mqtt 
  mqttClient.loop();

  // Get Serial commands from serial
  serialInput();

  // Is alive executed every 1 sec.
  if (millis() - previousMillis > 1000L) {
    previousMillis = millis();
#ifdef LED
    digitalWrite(EspLedBlue, !digitalRead(EspLedBlue));
#endif
    getLocalTime(&timeinfo, 0);
    
    // Wifi status
    wifiSt = WiFi.status();
    if ( (timeinfo.tm_min % 2==0) && timeinfo.tm_sec == 30) {
      if ((wifiSt != WL_CONNECTED) ) {
        wifiLost++;
        if (wifiLost == 2 ) {
          LOG("%s WiFi connection is lost. cnt:%dr",getPRoperty(HHMM).c_str(), wifiLost);
          WiFi.disconnect();
        }
        if (wifiLost == 4) {
          if (WiFi.reconnect()) {
            LOG("%s WiFi reconnect OK (%d).",getPRoperty(HHMM).c_str(), wifiLost);
            wifiLost = 0;
          }
        }
        if (wifiLost>10) wifiLost=0;
      }
    }

    // Prepare Display 
    qmsgstuct dmv {normal, day, 0, dsp};

    // Calibre is running (Start autoCalibration at begining) if dome present
    bool CalIsOk = false;
    if ( glassOk.get()) 
      CalIsOk = calibStp.CalibrationFinished();
    if (calibStp.autoCalRun!=0  || calibStp.turnCalRun!=0 || calibStp.testStepper) {
        LOG("pCal:%d tCal:%d Ccw:%c, cPos:%ld tgt:%ld moy:%ld ccwl:%ld ccwh:%ld acwl:%ld acwh:%ld", 
              calibStp.autoCalRun, 
              calibStp.turnCalRun, 
              ((calibStp.dirCcw)?('T'):('F')), 
              stepper.currentPosition(), 
              stepper.targetPosition() ,
              calibStp.mean,
              calibStp.ccwl, 
              calibStp.ccwh, 
              calibStp.acwl, 
              calibStp.acwh);
    }

    // Day or Night with 60sec hysteresis
    if (CalIsOk) {
      lux = analogRead(pinAd); 
      if (lux < Confwwm.config.dayTwilight ) {
        if (dayModeCnt<60) dayModeCnt++;
        else dayMode.set(false);
      } else {
        if (dayModeCnt>-60) dayModeCnt--;
        else dayMode.set(true);
      }
    }

    // NTQQ server test every 15sec
    if ((timeinfo.tm_sec%15)==0 && wifiSt==WL_CONNECTED ) {
      if ( strlen(Confwwm.config.mqttServer) > 2 ) {
        uint8_t tempo = Confwwm.myCrc8((uint8_t*)Confwwm.config.mqttServer, 59);
        if (tempo != mqttCrc8) {
          mqttCrc8 = tempo;
          MqttIn = "in"+String(frame.config.HostName);
          MqttOut = "out"+String(frame.config.HostName);
          mqttClient.setServer(Confwwm.config.mqttServer, Confwwm.config.mqttPort);
          mqttClient.setCallback(mqttCallback); 
        }
        if (!mqttClient.connected())
          mqttReconnect();
      } 
    }

    // Adafruit_LIS3DH
    if (lis3dhPresent) 
      lis3dhClick = lis3dh->getClick();

    if (glassOk.get()) {
      if ( CalIsOk && !stepper.isRunning() ) {
        // TOC TOC detected
        if (lis3dhClick!=0) {
          if (Confwwm.config.clickMode==TOC_1TB) { tocAction (1);}
          if (Confwwm.config.clickMode==TOC_5TB) { tocAction (5);}
          if (Confwwm.config.clickMode==TOC_HTB) { tocAction (timeinfo.tm_hour);}
        } else {
          if (Confwwm.config.actionNight==1 || dayMode.get() ) {
            if (Confwwm.config.actionNormal==ACT_T60 && timeinfo.tm_min==0 && timeinfo.tm_sec==0) clockAction (timeinfo.tm_hour,timeinfo.tm_min);
            if (Confwwm.config.actionNormal==ACT_T30 && timeinfo.tm_min%30==0 && timeinfo.tm_sec==0) clockAction (timeinfo.tm_hour,timeinfo.tm_min);
            if (Confwwm.config.actionNormal==ACT_T01 && timeinfo.tm_sec==0) clockAction (timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec);
            if ( (timeinfo.tm_hour%3)==0 && timeinfo.tm_min==0 && timeinfo.tm_sec==30 ) {
              LOG("%s -checkMean position:%ld and forceZero", getPRoperty(HHMM).c_str(), calibStp.checkMean());
              calibStp.forceZero(calibStp.checkMean());
            }
          }
        }
      } 
    } else {
      if (lis3dhClick!=0) {
        dmv.opt=param; // Display setup pages
      }
    }

    // Stop Lipo charging at 00:01 + Get easter eggs
    if (timeinfo.tm_hour==0 && timeinfo.tm_min==1 && timeinfo.tm_sec==55 && wdogPwr.isArmed()==false) { 
      LOG("%s +Start Charging + UTC time adjustement + WebImage",getPRoperty(HHMM).c_str());
      digitalWrite(pwrP5V, LOW);    // Stop lipo charging
      if ( !wdogPwr.isArmed()) {
        dmv.evt = web; // Get httpWebImage
      }
    }

    // Start Lipo changing ONLY SUNDAY AT 18:02.45
    if (timeinfo.tm_wday==0 && timeinfo.tm_hour==18 && timeinfo.tm_min==2 && timeinfo.tm_sec==45 ) { 
      LOG("%s +Start LIPO charging and get time",getPRoperty(HHMM).c_str());
      digitalWrite(pwrP5V, HIGH);    // START lipo chaging
    }

    // Task manager 
    if (dayMode.get()==false) dmv.opt=night;
    if (wdogPwr.isArmed()) dmv.md=stop;
    if (!xQueueSend(qDsp, &dmv, 0)) {
      xQueueReset(qDsp);
    }

    // test is dayMode or glass have changed
    if (mqttClient.connected()) {
      if (dayMode.isQ()) mqttPublish("Sensor", "Daylight", String(dayMode.get()));
      if (glassOk.isQ()) mqttPublish("Sensor", "GlassP", String(glassOk.get())); 
    }
  } // second
}