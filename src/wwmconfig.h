#ifndef wwmconfig_h
#define wwmconfig_h

//---- Start Generated from src/wwmconfig.html file --- 2020-06-02 20:49:00.286675
const char HTTP_WWM[] PROGMEM = "<!DOCTYPE html><html><style>.cmt {text-align: justify;width: 30em;padding: 10px 8px 2px;display: block;}.r12 {text-align: right;width: 12em;display: inline-block;}.l12 {text-align: left;width: 12em;display: inline-block;}.l4 {text-align: center;width: 7em;display: inline-block;font-size: 80%;border: 1px dotted black;}legend {background-color: rgb(92, 81, 81);color: #fff;padding: 3px 6px;}.myBt {font-weight: bold;text-align: center;}.myBt:hover {background-color: #60ff05;}.myBttt:active {position: relative;}</style><script></script><head><meta charset='utf-8'><title>%%Title%%</title><meta content='width=device-width' name='viewport'></head><body><center><header><h1 style='background-color: lightblue;'>%%Title%%</h1></header><form action='/wwm' method='post' ><fieldset><legend> Current state </legend><div class='cmt'>The Wifi link is %%Wifist%%, and the server MQTT is %%Mqttst%%. Stepping motor position is at %%Step%% step. The Magnet north sensor is %%Hall%%. SPI Flash File System total/used %%FsFree%% Kbytes</div></fieldset><fieldset><legend> WWM Configurator </legend><div style=' width: 40em; display: block; '><div class='cmt'><b>WWM Configurator</b> allows to change some basic parameters. This parameters are stored into file configWWM.json located in EPROM embedded Filesystem.</div><div class='r12'><label>Feild </label></div><div class='l12'><label>Values</label></div><div class='l4'>Actual value</div><div class='cmt'><li>Append Network Time Protocol server name.</li></div><div class='r12'><label for='SrvNtp'>Server NTP :</label></div><div class='l12'><input type='text' name='SrvNtp'maxlength='30' value='%%SrvNtp%%'></div><div class='l4'>%%SrvNtp%%</div><div class='cmt'><li>Greenwich Mean Time offset and daylight saving time (DST) values are in seconds (i.e. -21600 equals -7 hours)</li></div><div class='r12'><label for='GmtOff'>GMT offset :</label></div><div class='l12'><input type='number' name='GmtOff' maxlength='25' value='%%GmtOff%%'></div><div class='l4'>%%GmtOff%%</div><div class='r12'><label for='DayOff'>Daylight Offset :</label></div><div class='l12'><input type='number' name='DayOff' value='%%DayOff%%' maxlength='7'></div><div class='l4'>%%DayOff%%</div></div><div style=' width: 40em; display: block; '><div class='cmt'><li>Stepper motor parameters in number of step (Number of steps per turn is overwritten during the full calibration processus)</li></div><div class='r12'><label for='StpTun'>Step per turn :</label></div><div class='l12'><input type='number' name='StpTun' value='%%StpTun%%' placeholder='4098' maxlength='7'></div><div class='l4'>%%StpTun%%</div><div class='r12'><label for='BadDir'>Bad direction after :</label></div><div class='l12'><input type='number' name='BadDir' value='%%BadDir%%' placeholder='600' maxlength='7'></div><div class='l4'>%%BadDir%%</div><div class='r12'><label for='InaStp'>Inaccuracy +/- :</label></div><div class='l12'><input type='number' name='InaStp' value='%%InaStp%%' placeholder='50' maxlength='4'></div><div class='l4'>%%InaStp%%</div></div><div style=' width: 40em; display: block; '><div class='cmt'><li>Sismograph sensitivity (1-255) and Action associate</li></div><div class='r12'><label for='SisSen'>Sensitivity :</label></div><div class='l12'><input type='number' name='SisSen' value='%%SisSen%%' placeholder='50' maxlength='4'></div><div class='l4'>%%SisSen%%</div><div class='r12'><label for='SisKno'>Action after knock :</label></div>%%SisKno%%<p style='line-height: 1.0; font-size: 0.8em;'>Remarks: In <b>current time</b> mode, one automatic movement will be triggered every half hour. </p></div><div style=' width: 40em; display: block; '><div class='cmt'><li>MQTT* Broker Profile Setting (Emty if not used)</li></div><div class='r12'><label for='BroAdr'>Broker Address :</label></div><div class='l12'><input type='text' name='BroAdr' maxlength='30' value='%%BroAdr%%'></div><div class='l4'>%%BroAdr%%</div><div class='r12'><label for='BroPor'>Broker Port :</label></div><div class='l12'><input type='number' name='BroPor' value='%%BroPor%%'  placeholder='1883' maxlength='7'></div><div class='l4'>%%BroPor%%</div><div class='r12'><label for='BroUse'>User name :</label></div><div class='l12'><input type='text' name='BroUse' value='%%BroUse%%' maxlength='30'></div><div class='l4'>%%BroUse%%</div><div class='r12'><label for='BroPwd'>Password :</label></div><div class='l12'><input type='text' name='BroPwd' value='%%BroPwd%%' maxlength='30'></div><div class='l4'>%%BroPwd%%</div><p style='line-height: 1.0; font-size: 0.8em;'>*Topics list and message are visible <a class='button' href='/help'>here</a></p></div><div style='display: block;'><div class='cmt'>Send current configuration to the %%Title%%</div><div class='button'><button type='submit' class=\"myBt\">Overwrite current parameters</button></div></div></form></fieldset><fieldset><legend> Commands </legend><td>- Select one command in the list :</td><form action='/wwm' method='post' name='Cmd'><select name='Cmd'><option value='none'>None</option><option value='param'>Show param. on ePaper</option><option value='p5turn'>Clockwise 5 turns</option><option value='n5turn'>Anti clockwise 5 turns</option><option value='calturn'>Calibrate step/turn</option><option value='restpos'>Calib. Rest position</option><option value='reboot'>Reboot</option></select><button type='submit'>Valid</button></form><p style='line-height: 1.0; font-size: 0.8em;'>Remarks: some operations take a long time to complete. Wait for any selected tasks to finish. </p></fieldset><div class='cmt'><b>Some specialist facilities</b> are available on page :<a class='button' href='/'>Specialist tools</a> and the list of commands (Http and Mqtt) are visible <a class='button' href='/help.html'>here</a>.</div></center></body></html>";
//---- len : 5812 bytes
//---- End Generated 

// CODE HERE ----------------------
long getNumber (const char* s) {
  char* p;
  long converted = strtol(s, &p, 10);
  if (*p)  return 25121962; // conversion failed because the input wasn't a number
  return converted;
}
String getSeletor1(int v) {
  switch (v){
    case 0: return "1_trun"; break;
    case 1: return "5_turns"; break;
    case 2: return "current time"; break;
    default: return "nothing";
  }
  return "";
}
String getTagSeletor1(int v) {
  if (Confwwm.config.clickMode==v)
    return "selected";
  return "";
}
// -------- WWM Web transformation into Get Set functions ------------- 
char strtmp [20];
void   setCmd(String str) { /* user for command */ 
  Serial.printf("setCmd[%s]\n\r", str.c_str());
  if      (str=="calturn"){ if (calibStp.manuCalRun==0 && calibStp.autoCalRun==0) calibStp.manuCalRun=1; }
  else if (str=="restpos"){ if (calibStp.manuCalRun==0 && calibStp.autoCalRun==0) calibStp.autoCalRun=1; }
  else if (str=="p5turn") { long v=stepper.currentPosition(); mqttPostionMsg(v+(calibStp.oneTurnStep*5), "Http");}
  else if (str=="n5turn") { long v=stepper.currentPosition(); mqttPostionMsg(v-(calibStp.oneTurnStep*5), "Http");}
  else if (str=="param")  { dsp dmv {0, start}; xQueueSend(qDsp, &dmv, 0);} 
  else if (str=="reboot") { ESP.restart();} 
  else { long v=getNumber(str.c_str()); if (v!=25121962) stepper.moveTo(v);} 
}
char * getBadDir(bool m) { snprintf(strtmp, sizeof(strtmp), "%ld", Confwwm.config.autoCalibationOut); return strtmp; }
void   setBadDir(String s) { Confwwm.config.autoCalibationOut=s.toInt(); }
char * getBroAdr(bool m) { return Confwwm.config.mqttServer; }
void   setBroAdr(String s) { strlcpy(Confwwm.config.mqttServer, s.c_str(), sizeof(Confwwm.config.mqttServer));}
char * getBroPor(bool m) { snprintf(strtmp, sizeof(strtmp), "%d", Confwwm.config.mqttPort); return strtmp; }
void   setBroPor(String s) { Confwwm.config.mqttPort=s.toInt(); }
char * getBroPwd(bool m) { return Confwwm.config.mqttPasswd; }
void   setBroPwd(String s) { strlcpy(Confwwm.config.mqttPasswd, s.c_str(), sizeof(Confwwm.config.mqttPasswd)); }
char * getBroUse(bool m) { return Confwwm.config.mqttUser; }
void   setBroUse(String s) { strlcpy(Confwwm.config.mqttUser, s.c_str(), sizeof(Confwwm.config.mqttUser));}
char * getDayOff(bool m) { snprintf(strtmp, sizeof(strtmp), "%d", Confwwm.config.daylightOffset_sec); return strtmp; }
char * getFsFree(bool m) { snprintf(strtmp, sizeof(strtmp), "%d/%d",  (SPIFFS.totalBytes()/1024), (SPIFFS.usedBytes()/1024)); return strtmp;}
void   setDayOff(String s) { Confwwm.config.daylightOffset_sec=s.toInt(); }
char * getGmtOff(bool m) { snprintf(strtmp, sizeof(strtmp), "%ld", Confwwm.config.gmtOffset_sec); return strtmp; }
void   setGmtOff(String s) { Confwwm.config.gmtOffset_sec=s.toInt();  }
char * getInaStp(bool m) { snprintf(strtmp, sizeof(strtmp), "%ld", Confwwm.config.autoCalibrationStep); return strtmp; }
void   setInaStp(String s) { Confwwm.config.autoCalibrationStep=s.toInt(); } 
char * getSisKno(bool m) { 
  String rt;
  if (m)
   rt="<div class='l12'><select name='SisKno'>\
             <option value='1tb' "+ getTagSeletor1(0) +">1 turn/back</option>\n\
             <option value='5tb' "+ getTagSeletor1(1) +">5 turn/back</option>\n\
             <option value='htb' "+ getTagSeletor1(2) +">current time = nbr. of turn</option>\n\
             <option value='0tb' "+ getTagSeletor1(3) +">Nothing</option>\n\
            </select></div><div class='l4'>"+getSeletor1(Confwwm.config.clickMode)+"</div>\n";
  else 
    rt=getSeletor1(Confwwm.config.clickMode);
  return (char*)rt.c_str();
}
void   setSisKno(String s) {
  if (s=="1tb") { Confwwm.config.clickMode=0;}
  else if (s=="5tb") {Confwwm.config.clickMode=1;}
  else if (s=="htb") {Confwwm.config.clickMode=2;}
  else if (s=="0tb") {Confwwm.config.clickMode=3;}
}
char * getSisSen(bool m) { snprintf(strtmp, sizeof(strtmp), "%d", Confwwm.config.clickThreshold); return strtmp; } 
void   setSisSen(String s) {int v=s.toInt(); if (v<1) v=1; if (v>255) v=255; Confwwm.config.clickThreshold=v; lis3dh->setClick(2, v);}
char * getSrvNtp(bool m) { return Confwwm.config.ntpServer;}
void   setSrvNtp(String s) { strlcpy(Confwwm.config.ntpServer, s.c_str(), sizeof(Confwwm.config.ntpServer));}
char * getStpTun(bool m) { snprintf(strtmp, sizeof(strtmp), "%ld", Confwwm.config.oneTurnInStep); return strtmp; }
void   setStpTun(String s) { Confwwm.config.oneTurnInStep=s.toInt(); }
char * getTitle(bool m) {  return frame.config.HostName;}
char * getWifist(bool m) { if (wifiSt == WL_CONNECTED) return (char*)"Connected"; else return (char*)"Not connected";}
char * getMqttst(bool m) { if (mqttClient.connected()) return (char*)"Connected"; else return (char*)"Not connected";}
char * getStepper(bool m) {snprintf(strtmp, sizeof(strtmp), "%ld", stepper.currentPosition()); return strtmp; }
char * getHall(bool m) { if (digitalRead(pinSp)==0) return (char*)"at good position"; else return (char*)"not at good position"; }

struct Equiv {
   char key[11];
   char* (*get_ptr)(bool); // True if html false if mqtt
   void  (*set_ptr)(String);
};

#define NBRITEMINDICO 19
Equiv dico[] ={
  {"%%Cmd%%"     ,  NULL      , &setCmd    },
  {"%%BadDir%%"  , &getBadDir , &setBadDir },
  {"%%BroAdr%%"  , &getBroAdr , &setBroAdr },
  {"%%BroPor%%"  , &getBroPor , &setBroPor },
  {"%%BroPwd%%"  , &getBroPwd , &setBroPwd },
  {"%%BroUse%%"  , &getBroUse , &setBroUse },
  {"%%DayOff%%"  , &getDayOff , &setDayOff },
  {"%%FsFree%%"  , &getFsFree , NULL       },
  {"%%GmtOff%%"  , &getGmtOff , &setGmtOff },
  {"%%InaStp%%"  , &getInaStp , &setInaStp },
  {"%%SisKno%%"  , &getSisKno , &setSisKno },
  {"%%SisSen%%"  , &getSisSen , &setSisSen },
  {"%%SrvNtp%%"  , &getSrvNtp , &setSrvNtp },
  {"%%StpTun%%"  , &getStpTun , &setStpTun },
  {"%%Title%%"   , &getTitle  ,  NULL      },
  {"%%Wifist%%"  , &getWifist ,  NULL      },
  {"%%Mqttst%%"  , &getMqttst ,  NULL      },
  {"%%Step%%"    , &getStepper,  NULL      },
  {"%%Hall%%"    , &getHall   ,  NULL      }
};

String getKey(int i) {
  return  "%%"+frame.server.argName(i)+"%%";
}

void callbackSetWwm(int i){
  for (int idx=0; idx<NBRITEMINDICO; idx++) {
    if (getKey(i)==dico[idx].key && dico[idx].set_ptr != NULL) {
      (*dico[idx].set_ptr)(frame.server.arg(i));
      return;
    }
  }
}

String callbackGetWwm(int i, bool m){
  for (int idx=0; idx<NBRITEMINDICO; idx++) {
    if (getKey(i)==dico[idx].key  && dico[idx].get_ptr!=NULL) {
      return (*dico[idx].get_ptr)(m);
    }
  }
  return "";
}

String sentHtmlWwm(){
  Serial.println(frame.textNotFound());
  // get if action
  if (frame.server.method() == HTTP_POST || frame.server.method() == HTTP_GET) {
    for (uint8_t i=0; i<frame.server.args(); i++) {
      Serial.printf("sentHtmlEau(Post or Get)  Arg->[%s]:[%s]\n\r", frame.server.argName(i).c_str(), frame.server.arg(i).c_str() );
      if (frame.server.args()==1 && frame.server.arg(i).isEmpty()) // Received a Get like "Cmd=" with only one argument => return "answer" As text
        return callbackGetWwm(i, false); // Return immediate value
      else
        callbackSetWwm(i);
    }
    if (Confwwm.isCcrChanged())
      Confwwm.saveConfigurationJson();
  } 
  // Return current values with substitution
  String rt = HTTP_WWM;
  for (int idx=0; idx<NBRITEMINDICO; idx++){
    if (dico[idx].get_ptr!=NULL) rt.replace(dico[idx].key, (*dico[idx].get_ptr)(true) );
  }
  return rt;
}

#endif