#ifndef wwmconfig_h
#define wwmconfig_h

//---- Start Generated from src/wwmconfig.html file --- 2022-03-05 17:53:18.417893
const char HTTP_WWM[] PROGMEM = "<!DOCTYPE html><html><style>.cmt {text-align: justify;width: 30em;padding: 10px 8px 2px;display: block;}.r12 {text-align: right;width: 12em;display: inline-block;}.l12 {text-align: left;width: 12em;display: inline-block;}.l4 {text-align: center;width: 10em;display: inline-block;font-size: 80%;border: 1px dotted black;overflow: hidden;max-width: 20ch;}legend {background-color: rgb(92, 81, 81);color: #fff;padding: 3px 6px;}.myBt {font-weight: bold;text-align: center;}.myBt:hover {background-color: #60ff05;}.myBttt:active {position: relative;}</style><script></script><head><meta charset='utf-8'><title>%%Title%%</title><meta content='width=device-width' name='viewport'></head><body><center><header><h1 style='background-color: lightblue;'>%%Title%%</h1></header><form action='/wwm' method='post' ><fieldset><legend> Current state </legend><div class='cmt'>The Wifi link is %%Wifist%%, and the server MQTT is %%Mqttst%%. Stepping motor position is at %%Step%% step. The Magnet north sensor is %%Hall%%. The glass dome is %%GlassP%%. The photo-resistor module is at %%PhotoR%% (0:dark 4095:bright) .SPI Flash File System total/used %%FsFree%% Kbytes</div></fieldset><fieldset><legend> WWM Configurator </legend><div style=' width: 40em; display: block; '><div class='cmt'><b>WWM Configurator</b> allows to change some basic parameters. This parameters are stored into file configWWM.json located in EPROM embedded Filesystem.</div><div class='r12'><label>Feild </label></div><div class='l12'><label>Values</label></div><div class='l4'>Actual value</div><div class='cmt'><li>Append Network Time Protocol server name.</li></div><div class='r12'><label for='SrvNtp'>Server NTP :</label></div><div class='l12'><input type='text' name='SrvNtp'maxlength='30' value='%%SrvNtp%%'></div><div class='l4'>%%SrvNtp%%</div><div class='cmt'><li>Greenwich Mean Time offset and daylight saving time (DST) values are in seconds (i.e. -21600 equals -7 hours)</li></div><div class='r12'><label for='GmtOff'>GMT offset :</label></div><div class='l12'><input type='number' name='GmtOff' maxlength='25' value='%%GmtOff%%'></div><div class='l4'>%%GmtOff%%</div><div class='r12'><label for='DayOff'>Daylight Offset :</label></div><div class='l12'><input type='number' name='DayOff' value='%%DayOff%%' maxlength='7'></div><div class='l4'>%%DayOff%%</div></div><div style=' width: 40em; display: block; '><div class='cmt'><li>Stepper motor parameters in number of step (Number of steps per turn is overwritten during the full calibration processus)</li></div><div class='r12'><label for='StpTun'>Step per turn :</label></div><div class='l12'><input type='number' name='StpTun' value='%%StpTun%%' placeholder='4098' maxlength='7'></div><div class='l4'>%%StpTun%%</div><div class='r12'><label for='BadDir'>Bad direction after :</label></div><div class='l12'><input type='number' name='BadDir' value='%%BadDir%%' placeholder='600' maxlength='7'></div><div class='l4'>%%BadDir%%</div><div class='r12'><label for='InaStp'>Inaccuracy +/- :</label></div><div class='l12'><input type='number' name='InaStp' value='%%InaStp%%' placeholder='50' maxlength='4'></div><div class='l4'>%%InaStp%%</div></div><div style=' width: 40em; display: block; '><div class='cmt'><li>WatchWinderMax general behaviour during the day</li></div><div class='r12'><label for='ActNor'>Action normal :</label></div>%%ActNor%%<div class='r12'><label for='MovNig'> Move even at night :</label></div>%%MovNig%%<p style='line-height: 1.0; font-size: 0.8em;'>Comment: In <b>Turn hours</b> one turn by hour <b>Clock hand turn</b> movement every 1 minutes.</p></div><div style=' width: 40em; display: block; '><div class='cmt'><li>Sensor Sensitivity and Action associate</li></div><div class='r12'><label for='DaySen'> Twilight (0-4096) :</label></div><div class='l12'><input type='number' name='DaySen' value='%%DaySen%%' placeholder='400' maxlength='4'></div><div class='l4'>%%DaySen%%</div><div class='r12'><label for='SisSen'>Sismograph (0-255) :</label></div><div class='l12'><input type='number' name='SisSen' value='%%SisSen%%' placeholder='50' maxlength='4'></div><div class='l4'>%%SisSen%%</div><div class='r12'><label for='SisKno'>Action after knock :</label></div>%%SisKno%%<p style='line-height: 1.0; font-size: 0.8em;'>Comment: In <b>current time</b> mode, turn alternately in the direction clockwise or anti-clockwise  </p></div><div style=' width: 40em; display: block; '><div class='cmt'><li>MQTT* Broker Profile Setting (Emty if not used)</li></div><div class='r12'><label for='BroAdr'>Broker Address :</label></div><div class='l12'><input type='text' name='BroAdr' maxlength='62' value='%%BroAdr%%'></div><div class='l4'>%%BroAdr%%</div><div class='r12'><label for='BroPor'>Broker Port :</label></div><div class='l12'><input type='number' name='BroPor' value='%%BroPor%%'  placeholder='1883' maxlength='7'></div><div class='l4'>%%BroPor%%</div><div class='r12'><label for='BroUse'>User name :</label></div><div class='l12'><input type='text' name='BroUse' value='%%BroUse%%' maxlength='30'></div><div class='l4'>%%BroUse%%</div><div class='r12'><label for='BroPwd'>Password :</label></div><div class='l12'><input type='text' name='BroPwd' value='%%BroPwd%%' maxlength='30'></div><div class='l4'>%%BroPwd%%</div><p style='line-height: 1.0; font-size: 0.8em;'>*Topics list and message are visible <a class='button' href='/help.html'>here</a></p></div><div style=' width: 40em; display: block; '><div class='cmt'><li>HTTP* Get server for web[0-9].jpg at 00:00 (Emty if not used)</li></div><div class='r12'><label for='HtpAdr'>Server Address :</label></div><div class='l12'><input type='text' name='HtpAdr' maxlength='62' value='%%HtpAdr%%'></div><div class='l4'>%%HtpAdr%%</div></div><div style='display: block;'><div class='cmt'>Send current configuration to the %%Title%%</div><div class='button'><button type='submit' class=\"myBt\">Overwrite current parameters</button></div></div></form></fieldset><fieldset><legend> Commands </legend><td>- Select one command in the list :</td><form action='/wwm' method='post' name='Cmd'><select name='Cmd'><option value='none'>None</option><option value='param'>Show param. on ePaper</option><option value='p5turn'>Clockwise 5 turns</option><option value='n5turn'>Anti clockwise 5 turns</option><option value='calturn'>Calibrate step/turn</option><option value='restpos'>Calib. Rest position</option><option value='reboot'>Reboot</option></select><button type='submit'>Valid</button></form><p style='line-height: 1.0; font-size: 0.8em;'>Remarks: some operations take a long time to complete. Wait for any selected tasks to finish. </p></fieldset><div class='cmt'><b>Some specialist facilities</b> are available on page :<a class='button' href='/'>Specialist tools</a> and the list of commands (Http, Mqtt and FS) are visible <a class='button' href='/help.html'>here</a>.</div></center></body></html>";
//---- len : 6927 bytes
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
String getSeletor2(int v){
  switch(v) {
    case 0: return "Turn hours"; break;
    case 1: return "Turn half an hour"; break;
    case 2: return "Clock hand turn"; break;
    default: return "Nothing"; break;
  }
  return "";
}
String getTagSeletor2(int v) {
  if (Confwwm.config.actionNormal==v)
    return "selected";
  return "";
}
// -------- WWM Web transformation into Get Set functions ------------- 
char strtmp [20];
void   setCmd(String str) { /* user command */ 
 // Serial.printf("setCmd[%s]\n\r", str.c_str());
  if      (str=="calturn"){ if ( calibStp.autoCalRun==0 && calibStp.turnCalRun==0) calibStp.turnCalRun=1; }
  else if (str=="restpos"){ if ( calibStp.autoCalRun==0 && calibStp.turnCalRun==0) calibStp.autoCalRun=1; }
  else if (str=="p5turn") { long v=stepper.currentPosition(); mqttPostionMsg(v+(Confwwm.config.oneTurnInStep*5), "Http");}
  else if (str=="n5turn") { long v=stepper.currentPosition(); mqttPostionMsg(v-(Confwwm.config.oneTurnInStep*5), "Http");}
  else if (str=="param")  { qmsgstuct dmv {normal, param, 0, dsp}; xQueueSend(qDsp, &dmv, 0);} 
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
            <option value='1tb' "+ getTagSeletor1(TOC_1TB) +">1 turn / back</option>\n\
            <option value='5tb' "+ getTagSeletor1(TOC_5TB) +">5 turns / back</option>\n\
            <option value='htb' "+ getTagSeletor1(TOC_HTB) +">hour = nbr. of turn</option>\n\
            <option value='0tb' "+ getTagSeletor1(TOC_0TB) +">Nothing</option>\n\
            </select></div><div class='l4'>"+getSeletor1(Confwwm.config.clickMode)+"</div>\n";
  else 
    rt=getSeletor1(Confwwm.config.clickMode);
  return (char*)rt.c_str();
}
void   setSisKno(String s) {
  if (s=="1tb") { Confwwm.config.clickMode=TOC_1TB;}
  else if (s=="5tb") {Confwwm.config.clickMode=TOC_5TB;}
  else if (s=="htb") {Confwwm.config.clickMode=TOC_HTB;}
  else if (s=="0tb") {Confwwm.config.clickMode=TOC_0TB;}
}
char * getActNor(bool m) {
  String rt;
  if (m) 
    rt = "<div class='l12'><select name='ActNor'>\
            <option value='t60' "+ getTagSeletor2(ACT_T60) +">Turn hours</option>\n\
            <option value='t30' "+ getTagSeletor2(ACT_T30) +">Turn half an hour</option>\n\
            <option value='t01' "+ getTagSeletor2(ACT_T01) +">Clock hand turn</option>\n\
            <option value='t00' "+ getTagSeletor2(ACT_T00) +">Nothing</option>\n\
            </select></div><div class='l4'>"+getSeletor2(Confwwm.config.actionNormal)+"</div>\n";
  else 
    rt=getSeletor2(Confwwm.config.actionNormal);
  return (char*)rt.c_str();
}
void  setActNor(String s) {
   if (s=="t60") { Confwwm.config.actionNormal=ACT_T60;}
   else if  (s=="t30") { Confwwm.config.actionNormal=ACT_T30;}
   else if  (s=="t01") { Confwwm.config.actionNormal=ACT_T01;}
   else if  (s=="t00") { Confwwm.config.actionNormal=ACT_T00;}
}
String getTagSelector3(int v) { return ((Confwwm.config.actionNight==v)?("checked"):(""));}
String getSelector3() { return ((Confwwm.config.actionNight)?("yes"):("no")); }
char * getMovNig(bool m) {
  String rt;
  if (m)
    rt = "<div class='l12'>\
          <input type='radio' name='MovNig' value='yes' " + getTagSelector3(1) + "> yes \
          <input type='radio' name='MovNig' value='no' " + getTagSelector3(0) + "> no </div> \
    <div class='l4'>"+ getSelector3() +"</div>\n";
  else
    rt=getSelector3();
  return (char*)rt.c_str();
}
void setMovNig(String s) {
  if (s=="yes") Confwwm.config.actionNight=1;
  else Confwwm.config.actionNight=0;
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
char * getDaySen(bool m) { snprintf(strtmp, sizeof(strtmp), "%d", Confwwm.config.dayTwilight); return strtmp; }
void   setDaySen(String s) { Confwwm.config.dayTwilight=s.toInt(); }
char * getGlassP(bool m) { if (digitalRead(pinGl)==0) return (char*)"at correct place"; else return (char*)"not at correct place"; }
char * getPhotoR(bool m) { snprintf(strtmp, sizeof(strtmp), "%d", analogRead(pinAd)); return strtmp; }
char * getHtpAdr(bool m) { return Confwwm.config.ImageHttpEa;}
void   setHtpAdr(String s) { strlcpy(Confwwm.config.ImageHttpEa, s.c_str(), sizeof(Confwwm.config.ImageHttpEa)); }

struct Equiv {
   char key[11];
   char* (*get_ptr)(bool); // True if html false if mqtt
   void  (*set_ptr)(String);
};

//command to extract "python3 extra_script.py -f src/wwmconfig.html"
// Key list   : ['MovNig', 'ActNor', 'BadDir', 'BroAdr', 'BroPor', 'BroPwd', 'BroUse', 'DayOff', 'DaySen', 'FsFree', 'GlassP', 'GmtOff', 'Hall', 'HtpAdr', 'InaStp', 'Mqttst', 'PhotoR', 'SisKno', 'SisSen', 'SrvNtp', 'Step', 'StpTun', 'Title', 'Wifist']
// Number Key : 24
// Max Key len: 6
#define NBRITEMINDICO 25 //  Number Key + 1 because CMD is append

Equiv dico[] ={
  {"%%Cmd%%"     ,  NULL      , &setCmd    },
  {"%%BadDir%%"  , &getBadDir , &setBadDir },
  {"%%BroAdr%%"  , &getBroAdr , &setBroAdr },
  {"%%BroPor%%"  , &getBroPor , &setBroPor },
  {"%%BroPwd%%"  , &getBroPwd , &setBroPwd },
  {"%%BroUse%%"  , &getBroUse , &setBroUse },
  {"%%DayOff%%"  , &getDayOff , &setDayOff },
  {"%%DaySen%%"  , &getDaySen , &setDaySen },
  {"%%FsFree%%"  , &getFsFree , NULL       },
  {"%%GlassP%%"  , &getGlassP , NULL       },
  {"%%GmtOff%%"  , &getGmtOff , &setGmtOff },
  {"%%InaStp%%"  , &getInaStp , &setInaStp },
  {"%%PhotoR%%"  , &getPhotoR , NULL       },
  {"%%SisKno%%"  , &getSisKno , &setSisKno },
  {"%%SisSen%%"  , &getSisSen , &setSisSen },
  {"%%SrvNtp%%"  , &getSrvNtp , &setSrvNtp },
  {"%%StpTun%%"  , &getStpTun , &setStpTun },
  {"%%Title%%"   , &getTitle  ,  NULL      },
  {"%%Wifist%%"  , &getWifist ,  NULL      },
  {"%%Mqttst%%"  , &getMqttst ,  NULL      },
  {"%%Step%%"    , &getStepper,  NULL      },
  {"%%Hall%%"    , &getHall   ,  NULL      },
  {"%%HtpAdr%%"  , &getHtpAdr ,  &setHtpAdr},
  {"%%ActNor%%"  , &getActNor ,  &setActNor},
  {"%%MovNig%%"  , &getMovNig ,  &setMovNig},
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
      Serial.printf("sentHtml(Post or Get)  Arg->[%s]:[%s]\n\r", frame.server.argName(i).c_str(), frame.server.arg(i).c_str() );
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