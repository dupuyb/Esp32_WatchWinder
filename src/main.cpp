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

// mqtt 
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Power supply-survey
#define pinAo 36 // ADC1_0 A 
#define pin5V 4 // Relay power-supply is on at 1
#define pinSp 5  // Hall cadran position detector
#define pinT8 33 // Touch sensor
#define pinT9 32
uint16_t vddUsb = 4095;
uint16_t lowLimit = 0xE00;

const char *version = "0.6.4";
const char *HOSTNAME = "WatchWinderMax";
int wifiSt = -1;

// Time facilities
struct tm timeinfo;                  // time struct

// for SPI pin definitions see e.g. C:\Users\xxx\Documents\Arduino\hardware\espressif\esp32\variants\lolin32\pins_arduino.h
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

// Stepper Gear reduction	1/64 (see note)
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
uint8_t LIS3DH_Click = 0;
bool LIS3DH_Present;

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
DspEpaper eDsp(&display, &stepper, &timeinfo, &calibStp, &LIS3DH_Click, &wifiSt, (char *)version);
// Task
static int app_cpu = 0; // Updated by setup() 
QueueHandle_t qDsp = xQueueCreate(5, sizeof(dsp));
TaskHandle_t epaperCxHandle = NULL;
TaskHandle_t wifiCxHandle = NULL;
TaskHandle_t stepperCxHandle = NULL;
#define DSP_TASK_PRIORITY 5
#define WIFI_TASK_PRIORITY 7
#define MAIN_TASK_PRIORITY  10
#define STEPPER_TASK_PRIORITY 12 // Low priority numbers denote low priority tasks

// Debug
int8_t cmd;

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

// Display Task waitting message in qDsp queue
void IRAM_ATTR epaperTask(void *pvParameter) {
  while (1) { 
    dsp dmv;
    xQueueReceive (qDsp, &dmv, portMAX_DELAY);
    eDsp.mainLoop(dmv);
    yield();
  }
}

//-------------------------------------------------
void IRAM_ATTR wifiTask(void *pvParameter) {
  while (1) {
    // Wifi is running
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("wifiTask started");
      frame.externalHtmlTools="Specific home page is visible at :<a class='button' href='/wwm'>W.W.M Page</a>";
      frame.setup(HOSTNAME);
    }
    yield();
    vTaskDelay(5);
  }
}

// Increment or Decrement step
void IRAM_ATTR stepperTask(void *pvParameter) {
  Serial.println("stepperTask started");
  while (1) {
    if (stepper.run()){
      stepper.enableOutputs();
    } else {
      stepper.disableOutputs();
    }
    vTaskDelay(1);
  }
}

// Interrupt if Hall  sensor A3114 detects the cadran magnet
void IRAM_ATTR positionIN() {
  calibStp.pIn2=calibStp.pIn1;
  calibStp.pIn1=stepper.currentPosition();
}
void IRAM_ATTR positionOUT() {
  calibStp.pOut2=calibStp.pOut1;
  calibStp.pOut1=stepper.currentPosition();
}
void IRAM_ATTR positionCHG(){
  int val = digitalRead(pinSp);
  if (val==HIGH) positionOUT(); 
  else positionIN();
}

void httpGetFile (String name){
  HTTPClient http;  
  Serial.println("Start http request:"+name);
  http.begin( Confwwm.config.ImageHttpEa + name);
  http.addHeader("Content-Type", "text/plain");                       //Specify content-type header
  int httpResponseCode = http.POST("POSTING from WatchWinderMax");   //Send the actual POST request
  if(httpResponseCode>0) {
    String response = http.getString();                               //Get the response to the request
      if (httpResponseCode==200){   
      File file = SPIFFS.open(name, "w");
      if (!file) {
        Serial.println("Can't write in image file:");
        Serial.println(name);
      } else {
        file.print(response),
        file.close();
      }
    }
  } else {
    Serial.print("Error on sending POST: ");
    Serial.println(httpResponseCode);
  }
  http.end();  //Free resources
  Serial.println("httpGetFile is finished.");
}

//! adapter topic message ---------------------------------
String MqttIn;
String MqttOut;
bool mqttUsed = false;

void mqttPublish(String src, String jkey, String jval){
  String msg;
  DynamicJsonDocument json(80);
  json["From"] = src;
  json[jkey] = jval;
  serializeJson(json, msg);
  Serial.printf("MqttPublish Topic[%s] msg[%s]\n\r", MqttOut.c_str(), msg.c_str());
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

void mqttPostionMsg(long pos, String src){
  if (mqttClient.connected()) {
    mqttPublish(src, "Cmd", String(pos));
  }
  stepper.moveTo(pos);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT callback ["); Serial.print(topic); Serial.println("] ");
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
    Serial.printf("MQTT Callback [%s]:[%s]\n\r", key.c_str(), val.c_str() );
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
  Serial.println(output.c_str());
  if (empty){
    mqttPublishAll();
  } 
}

bool mqttReconnect() {
  // Loop until we're reconnected
  if (!mqttClient.connected()) {
    String clientId = frame.config.HostName;
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str())) {
      // Once connected, publish an announcement... and resubscribe 
      mqttPublish("Mqtt", "Reconnection", "true");
      mqttClient.subscribe(MqttIn.c_str());
    } else {
      return false;
    }
  }
  return true;
}

void setup(void) {
  // Set Serial ---------------------------------------
  Serial.begin(115200);
#ifdef LED
  pinMode(EspLedBlue, OUTPUT);     // Led is BLUE at statup
  digitalWrite(EspLedBlue, HIGH);  // After 5 seconds blinking indicate WiFI ids OK
#endif
  // Arm power ----------------------------------------
  pinMode(pin5V, OUTPUT);
  digitalWrite(pin5V, HIGH);    // initial state power ON
  // Set Power-supply survey & position capture
  pinMode(pinAo, INPUT);
  pinMode(pinSp, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinSp), positionCHG, CHANGE);
  // Load configuration
  Confwwm.loadConfigurationJson();
  // TEST A FAIRE
  //! WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  // Set Task priority --------------------------------
  TaskHandle_t h = xTaskGetCurrentTaskHandle();   
  app_cpu = xPortGetCoreID(); // Quelle UC ?   
  vTaskPrioritySet(h,MAIN_TASK_PRIORITY);
  // setup the display --------------------------------
  // Adapte to GDEW0213I5F 212x104, 2.13inch flexible E-Ink display on ePaperESP32 Driver Board
  spi.begin(PIN_SPI_SCK, -1, PIN_SPI_DIN, PIN_SPI_CS);
  display.init();
  //! Mode Landscape /!\ _height = WIDTH; _width = HEIGHT;
  display.setRotation(1);
  // first message welcome message
  xTaskCreatePinnedToCore( &epaperTask, "epaperTask", 8096, NULL, DSP_TASK_PRIORITY, nullptr, ((app_cpu==0)?1:0));   
  dsp dmv {0, start};
  xQueueSend(qDsp, &dmv, 0);
  // WiFi Task ----------------------------------------
  if (app_cpu) disableCore0WDT();
  else disableCore1WDT();
  xTaskCreatePinnedToCore(&wifiTask, "wifiTask", 8096,  NULL, WIFI_TASK_PRIORITY,  &wifiCxHandle, app_cpu);
  // Init Stepper And StepperTask ----------------------
  calibStp.init();
  xTaskCreatePinnedToCore( &stepperTask, "stepperTask", 3000,  NULL, STEPPER_TASK_PRIORITY, &stepperCxHandle, ((app_cpu==0)?1:0));
  // Wire i2c for Lis30H --------------------------------
  Wire.begin(pinSDA, pinSCL);
  lis3dh = new Adafruit_LIS3DH(&Wire);
#ifdef I2CTST
  scanI2C();
#endif
  if (!lis3dh->begin(0x19)){ // change this to 0x18 for alternative i2c address
    LIS3DH_Present = false;
    Serial.println("//!\\ Error LIS3DH not found!");
  } else {
    LIS3DH_Present = true;
    Serial.println("LIS3DH found!");
  }
  lis3dh->setRange(LIS3DH_RANGE_2_G); // 2, 4, 8 or 16 G!
  // Adjust threshhold, higher numbers are less sensitive
  lis3dh->setClick(2, Confwwm.config.clickThreshold);
  // End of sterring --------------------------------------
  sleep(2);
  yield();
  Serial.println(">> start epaperTask");
}

void tocAction (int t){
  long absoluP = 0;
  if ((int)(stepper.currentPosition()/10) == 0) 
    absoluP=calibStp.oneTurnStep * t;
  mqttPostionMsg(absoluP, "Knock");
  stepper.moveTo(absoluP);
} 
void clockAction (int h, int m){
  long absoluP = 0;
  if ( m==0)  
    absoluP = calibStp.oneTurnStep * h;
  mqttPostionMsg(absoluP, "Clock");
  stepper.moveTo(absoluP);
} 

//-------------  Main task  ------------------------------------
int WifiError=0;
void loop(void) {

  // Get voltage 
  vddUsb = analogRead(pinAo);
  if (vddUsb < lowLimit) {
    stepper.disableOutputs();
    dsp dmv {0, stop};
    xQueueSend(qDsp, &dmv, 0);
  }

  if (eDsp.screen==STOP)
     digitalWrite(pin5V, LOW);  

  // Wifi is running
  if (wifiSt == WL_CONNECTED) {
    WifiError=0;
    // Delete WifiTask
    if (wifiCxHandle != NULL) {
      // Append /wwm access html 
      frame.server.on("/wwm", [](){
        frame.server.send(HTTP_CODE_OK, "text/html", sentHtmlWwm());
      });
      vTaskDelete(wifiCxHandle);
      Serial.println(">> delete wifiTask ");
      // Init time
      configTime(Confwwm.config.gmtOffset_sec, Confwwm.config.daylightOffset_sec, Confwwm.config.ntpServer); //init and get the time
      wifiCxHandle = NULL;
      // Start MQTT client
      if ( strlen(Confwwm.config.mqttServer)>2 ) {
        MqttIn = "in"+String(frame.config.HostName);
        MqttOut = "out"+String(frame.config.HostName);
        mqttClient.setServer(Confwwm.config.mqttServer, Confwwm.config.mqttPort);
        mqttClient.setCallback(mqttCallback); 
        mqttUsed=true;
      }
    } 
  } else {
    // Append reconnection
    WifiError++;
    // Resart Wifi connect if lost more than 1 hour
    if ( wifiCxHandle == NULL && WifiError>3600 ){
      xTaskCreatePinnedToCore(&wifiTask, "wifiTask", 8096,  NULL, WIFI_TASK_PRIORITY,  &wifiCxHandle, app_cpu);
    }
  }

  // Call wifi loop % mqtt
  frame.loop();

  // Mqtt 
  mqttClient.loop();

  // Get Serial commands
	while (Serial.available() > 0) {
	  uint8_t c = (uint8_t)Serial.read();
	  if (c != 13 && c != 10 ) {
      cmd = c;
    } else {
      if (c==13) {
        if (cmd=='h') { Serial.println("- Help info=>\n\r  -i: info\n\r  -r: reboot\n\r  -d: display\n\r  -s: vddUsbLow\n\r  -q/w: Pwr hi/low\n\r  -t: http\n\r  -c/C: Calib.m/A\n\r  -0/1/2: Move stepper\n\r  -g/p: Debug\n\r");}
			  else if (cmd=='i') { Serial.printf("Heap:%u IP:%s \n\r",ESP.getFreeHeap(), WiFi.localIP().toString().c_str() ); }
        else if (cmd=='r') { ESP.restart(); }
        else if (cmd=='s') { Serial.println("vddUsl low"); vddUsb=3000; }
        else if (cmd=='t') { cmd=' '; httpGetFile(Confwwm.config.ImageFileEa); }
        else if (cmd=='c') { Serial.println("Mode calibration manuel active.");  calibStp.manuCalRun=1; cmd=' '; }
        else if (cmd=='C') { Serial.println("Mode calibration auto.  active."); calibStp.autoCalRun=1; cmd=' '; }
        else if (cmd=='1') { stepper.moveTo(calibStp.oneTurnStep*2); cmd=' ';}
        else if (cmd=='0') { stepper.moveTo(0); cmd=' ';}
        else if (cmd=='2') { stepper.moveTo(calibStp.oneTurnStep*5), cmd=' '; }
        else if (cmd=='g') { Serial.printf("Pos:%ld  sp:%f isRun:%d \n\r",stepper.currentPosition(), stepper.speed(), stepper.isRunning());  cmd=' ';}
        else if (cmd=='p') { Serial.printf("%d manuCalRun=%d Click=%d Pwr=%d \n\r", timeinfo.tm_sec, calibStp.manuCalRun, LIS3DH_Click , vddUsb);  cmd=' '; }
        else { Serial.printf("Stop serial: %s \n\r",version); }
      }
		}
  }

  // Is alive executed every 1 sec.
  if (millis() - previousMillis > 1000L) {
    previousMillis = millis();
#ifdef LED
    digitalWrite(EspLedBlue, !digitalRead(EspLedBlue));
#endif
   if (eDsp.screen==STOP)
      return;

    getLocalTime(&timeinfo, 0);

    if (mqttUsed && (timeinfo.tm_sec%15)==0 ) {
      if (!mqttClient.connected())
        mqttReconnect();
    }

    // Calibre is running
    bool CalIsOk = calibStp.CalibrationFinished();

      // Adafruit_LIS3DH
    if (LIS3DH_Present) 
      LIS3DH_Click = lis3dh->getClick();

    if ( CalIsOk && !stepper.isRunning() ) {
      // TOC TOC detected
      if (LIS3DH_Click!=0) {
        //Serial.printf("LIS3DH_Click:0x%X \n\r", LIS3DH_Click);
        if (Confwwm.config.clickMode==0) { tocAction (1);}
        if (Confwwm.config.clickMode==1) { tocAction (5);}
        if (Confwwm.config.clickMode==2) { tocAction (timeinfo.tm_hour);}
      } else {
        if (Confwwm.config.clickMode==2 && timeinfo.tm_min%30==0 && timeinfo.tm_sec==0) { 
          clockAction (timeinfo.tm_hour,timeinfo.tm_min);
        }
      }
    } 
  
    wifiSt = WiFi.status();

    // Push to Display queue. from 0 to 86399 second
    int tsec =  timeinfo.tm_hour * 3600 + timeinfo.tm_min * 60 + timeinfo.tm_sec;
    if (tsec==0) // at // 00:00 Get easter eggs
      httpGetFile(Confwwm.config.ImageFileEa);
    dsp dmv {tsec, normal};
    if (!xQueueSend(qDsp, &dmv, 0)) 
       xQueueReset(qDsp);
  }
}

