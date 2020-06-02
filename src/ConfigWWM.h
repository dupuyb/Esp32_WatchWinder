#ifndef ConfigWWM_h
#define ConfigWWM_h

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
  // Toc toc
  uint8_t clickThreshold;
  int clickMode;
  // Get image
  char ImageHttpEa[50];
  char ImageFileEa[30];
  // MQTT
  char mqttServer[30];
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
    // To Toc
    rootcfg["clickThreshold"] = config.clickThreshold;
    rootcfg["clickMode"] = config.clickMode;
    // Mqtt
    rootcfg["mqttServer"] = config.mqttServer;
    rootcfg["mqttPort"] = config.mqttPort;
    rootcfg["mqttUser"] = config.mqttUser;
    rootcfg["mqttPasswd"] = config.mqttPasswd;
    // 
    rootcfg["ImageHttpEa"] = config.ImageHttpEa; 
    rootcfg["ImageFileEa"] = config.ImageFileEa; 
    serializeJson(rootcfg, cfJeedomjson);
    file.print(cfJeedomjson);
    file.close();
    ccrConfig = getCcrConfig();
    return true;
  }

  void loadConfigurationJson() {
    SPIFFS.begin();
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
      config.clickThreshold = rootcfg["clickThreshold"] | 50;
      config.clickMode = rootcfg["clickMode"] | 0;
      // Mqtt
      strlcpy(config.mqttServer, rootcfg["mqttServer"] | "", sizeof(config.mqttServer));
      config.mqttPort = rootcfg["mqttPort"] | 1883;
      strlcpy(config.mqttUser, rootcfg["mqttUser"] | "", sizeof(config.mqttUser));
      strlcpy(config.mqttPasswd, rootcfg["mqttPasswd"] | "", sizeof(config.mqttPasswd));
      //! A Changer pas http://dududomo.duckdns.org/watchwinder
      strlcpy(config.ImageHttpEa, rootcfg["ImageHttpEa"] | "http://192.168.1.117/watchwinder", sizeof(config.ImageHttpEa));
      strlcpy(config.ImageFileEa, rootcfg["ImageFileEa"] | "/web0.jpg", sizeof(config.ImageFileEa));
      if (error) 
        saveConfigurationJson();
    }
    ccrConfig = getCcrConfig();
  }

   boolean isCcrChanged(){
     return (myCrc8((uint8_t*)&config, sizeof(config) - 1) != ccrConfig);
  }

  ConfigWWM (){ } 

private :
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