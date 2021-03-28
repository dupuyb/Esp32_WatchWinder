#ifndef ConfigWWM_h
#define ConfigWWM_h

// Action TocToc
#define TOC_1TB 0
#define TOC_5TB 1
#define TOC_HTB 2
#define TOC_0TB 3
// Action Normal
#define ACT_T60 0
#define ACT_T30 1
#define ACT_T01 2
#define ACT_T00 3

// configuration file jeedom
struct ConfigWwm {
  // Time
  long gmtOffset_sec;   
  int daylightOffset_sec; 
  char ntpServer[30];
  // Sensor & motor
  long oneTurnInStep;
  long autoCalibationOut;
  long autoCalibrationStep;
  // device behaviour
  int actionNormal;
  int actionNight;
  // Sensor day/night
  int dayTwilight;
  // Toc toc
  uint8_t clickThreshold;
  int clickMode;
  // Get image
  char ImageHttpEa[60];
  // MQTT
  char mqttServer[60];
  int mqttPort;
  char mqttUser[30];
  char mqttPasswd[30];
};

class ConfigWWM {
public:

  boolean saveConfigurationJson() {
    File file = SPIFFS.open(fileconfigwwm, "w");
    if (!file) {
       return false;
    }
    String cfJeedomjson;
    DynamicJsonDocument rootcfg(500);
    rootcfg["gmtOffset_sec"] = config.gmtOffset_sec;
    rootcfg["daylightOffset_sec"] = config.daylightOffset_sec;
    rootcfg["ntpServer"] = config.ntpServer;
    rootcfg["oneTurnInStep"] = config.oneTurnInStep;
    rootcfg["autoCalibationOut"] = config.autoCalibationOut;
    rootcfg["autoCalibrationStep"] = config.autoCalibrationStep; 
    // device behaviour
    rootcfg["actionNormal"] = config.actionNormal;
    rootcfg["actionNight"] = config.actionNight;
    // Sensor day/night
    rootcfg["twilight"] = config.dayTwilight;
    // To Toc
    rootcfg["clickThreshold"] = config.clickThreshold;
    rootcfg["clickMode"] = config.clickMode;
    // Mqtt
    rootcfg["mqttServer"] = config.mqttServer;
    rootcfg["mqttPort"] = config.mqttPort;
    rootcfg["mqttUser"] = config.mqttUser;
    rootcfg["mqttPasswd"] = config.mqttPasswd;
    // Http web image
    rootcfg["ImageHttpEa"] = config.ImageHttpEa; 
    serializeJson(rootcfg, cfJeedomjson);
    file.print(cfJeedomjson);
    file.close();
    ccrConfig = getCcrConfig();
    return true;
  }

  void loadConfigurationJson() {
    File file = SPIFFS.open(fileconfigwwm, "r");
    if (!file) {
      Serial.println(F("Configuration ConfigWWM file is absent."));
    } else {
      size_t size = file.size();
      std::unique_ptr<char[]> buf(new char[size]);
      file.readBytes(buf.get(), size);
      DynamicJsonDocument rootcfg(1024);
      auto error = deserializeJson(rootcfg, buf.get());
      // Clock
      config.gmtOffset_sec = rootcfg["gmtOffset_sec"] | -21600;
      config.daylightOffset_sec = rootcfg["daylightOffset_sec"] | 3600;
      strlcpy(config.ntpServer, rootcfg["ntpServer"] | "pool.ntp.org", sizeof(config.ntpServer));
      // WWM parmerters
      config.oneTurnInStep = rootcfg["oneTurnInStep"] | 4098;
      config.autoCalibationOut = rootcfg["autoCalibationOut"] | 600;
      config.autoCalibrationStep = rootcfg["autoCalibrationStep"] | 50;
      // device behaviour
      config.actionNormal = rootcfg["actionNormal"] | 1;
      config.actionNight = rootcfg["actionNight"] | 1;
      // Sensor day/night
      config.dayTwilight = rootcfg["twilight"] | 2048;
      config.clickThreshold = rootcfg["clickThreshold"] | 50;
      config.clickMode = rootcfg["clickMode"] | 0;
      // Mqtt
      strlcpy(config.mqttServer, rootcfg["mqttServer"] | "", sizeof(config.mqttServer));
      config.mqttPort = rootcfg["mqttPort"] | 1883;
      strlcpy(config.mqttUser, rootcfg["mqttUser"] | "", sizeof(config.mqttUser));
      strlcpy(config.mqttPasswd, rootcfg["mqttPasswd"] | "", sizeof(config.mqttPasswd));
      //! A Changer par https://dududomo.duckdns.org/watchwinder test local 192.168.2.117
      strlcpy(config.ImageHttpEa, rootcfg["ImageHttpEa"] | "https://dududomo.duckdns.org/watchwinder", sizeof(config.ImageHttpEa));
      if (error) 
        saveConfigurationJson();
    }
    ccrConfig = getCcrConfig();
  }

   boolean isCcrChanged(){
     return (myCrc8((uint8_t*)&config, sizeof(config) - 1) != ccrConfig);
  }

  ConfigWWM (){ } 

  uint8_t myCrc8(uint8_t * data, uint8_t count) {
    uint8_t result = 0xDF;
    while (count--) {
      result ^= *data;
      data++;
    }
    return result;
  }

  uint8_t getCcrConfig() {
    return myCrc8((uint8_t*)&config, sizeof(config) - 1);
  }

public:
  uint8_t ccrConfig;
  ConfigWwm config;

private:
  const char * fileconfigwwm="/configWWM.json";

};

#endif