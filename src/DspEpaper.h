#ifndef DspEpaper_h
#define DspEpaper_h

#include <GxEPD.h>                       // ePApaer
#include <GxGDEW0213I5F/GxGDEW0213I5F.h> // 2.13" b/w 104x212 flexible
#include "BitmapGraphics.h"
// FreeFonts from Adafruit_GFX
#include <Fonts/FreeSerif12pt7b.h>
#include <Fonts/FreeSerif18pt7b.h>
#include <Fonts/FreeSerifBold9pt7b.h>
//#include <Fonts/Org_01.h> //! too many font no enough memories  
#include <Fonts/FreeSerifBold24pt7b.h>
//#include <Fonts/FreeSerifItalic24pt7b.h>
#include "Monospaced_bold_8.h"
#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>
#include "ImageLoader.h"
#include <math.h>

// Available Font
//const GFXfont *fsi24 = &FreeSerifBold24pt7b; //&FreeSerifItalic24pt7b; //! Font use 0.7% memory We reduce the map or code
const GFXfont *fsb24 = &FreeSerifBold24pt7b;
const GFXfont *fsb9  = &FreeSerifBold9pt7b;
const GFXfont *f6    = &Monospaced_bold_8; //&Org_01;
const GFXfont *fs12  = &FreeSerif12pt7b;
const GFXfont *fs18  = &FreeSerif18pt7b;

// Image (bmp, jpeg) loader
ImageLoader imageLoader(GxGDEW0213I5F_WIDTH * 27); // 212/8 bytes + 104 Width

struct Msg {
  const GFXfont *font;
  int16_t x, y;
  String text;
};

#define STOP 99
#define NIGHT 59

typedef enum {   
  normal = 0, 
  start = 1, 
  stop = 2,
} mode;

typedef enum {
    day = 0,
    night = 1,
    param = 2,
} option;
typedef enum {
    dsp = 0,
    web = 1,
} event;

struct qmsgstuct {
  mode md;
  option opt;
  int dbx; // debug value if option = param(2)
  event evt; // Other action 0=ePaper  1=getHtpWebImage
} QMSG;

typedef enum {   
  ICON= 0, 
  WIFI= 1, 
  HHMM= 3,
  DMY = 4,
  MAC = 5,
  VER = 6,
  SPOS= 7,
  SSPE= 8,
  SISR= 9,
  SCAL= 10,
  DDMM= 11,
} prop;

class DspEpaper {
public:

    DspEpaper(GxEPD_Class* dsp, String (*ptrProp)(int) , tm* tii) {
        display=dsp;
        getProperty = ptrProp;
        timeinfo=tii;
    }

    uint16_t getColorTxt(bool inv) {
        return ((inv)?(GxEPD_WHITE):(GxEPD_BLACK));
    }
    uint16_t getColorBgd(bool inv) {
        return !getColorTxt(inv);
    }

    void epaperDate() {
        uint16_t box_x = 20;
        uint16_t box_y = 30;
        uint16_t box_w = GxGDEW0213I5F_HEIGHT - 40;
        uint16_t box_h = GxGDEW0213I5F_WIDTH - box_y;
        display->setTextColor(getColorTxt(invert));
        display->fillRect(box_x, box_y, box_w, box_h, getColorBgd(invert));
        display->setFont(fs18);
        display->setCursor(box_x, 55);
        display->print(getProperty(DMY).c_str());
        display->setFont(fsb24); //! OLD fsb24
//77 display->setTextSize(2); // Append x2
        display->setCursor(box_x + 30, 98);
        display->print(getProperty(HHMM).c_str());
        display->updateWindow(box_x, box_y, box_w, box_h, true);
    }

    void epaperIcon(const uint8_t *icon, const uint8_t w, const uint8_t h, bool refresh) {
        uint16_t box_x = GxGDEW0213I5F_HEIGHT - w;
        uint16_t box_y = 0;
        uint16_t box_w = w;
        uint16_t box_h = h;
        display->fillRect(box_x, box_y, box_w, box_h, getColorBgd(invert));
        if (icon!=NULL)
            display->drawBitmap(icon, box_x, box_y, box_w, box_h, getColorTxt(invert));
        if (refresh)
            display->updateWindow(box_x, box_y, box_w, box_h, true);
    }

    void epaperText(Msg msg[], int nbrMsg) {
        for (int i = 0; i < nbrMsg; i++){
            display->setFont(msg[i].font);
            display->setCursor(msg[i].x, msg[i].y);
            display->print(msg[i].text.c_str());
        }
    }

    bool imageExist(String filename) {
        bool ret=true;
        if (  SPIFFS.begin()==false || SPIFFS.exists(filename)==false ) {
          ret = false;
        }
        return ret;
    }

    bool epaperImage(String filename, Msg msg[], int nbrMsg) {
        bool ret=false;
        if (SPIFFS.begin() == true) {
            if (imageLoader.load(filename)) {
                display->drawBitmap(imageLoader.getBuffer(), 0, 0, GxGDEW0213I5F_HEIGHT, GxGDEW0213I5F_WIDTH, getColorTxt(invert));
                epaperText(msg, nbrMsg);
                ret=true;
            } 
        } 
        return ret;
    }

    String getNextImage(String n, int i) {
        String name = n+String(i)+String(".jpg");
        if (imageExist(name)) return name;
        return String("");
    }

    String getNextImage() {
        String name;
        if (iORw) { 
            String name;
            for (int i=imgIdx; i<9; i++) {
              name = getNextImage("/image", i);
              if (name.isEmpty()==false) break;
            }
            imgIdx=(((imgIdx+1)<9)?(imgIdx+1):(0));
        } else { 
            String name;
            for (int i=webIdx; i<9; i++) {
              name = getNextImage("/web", i);
              if (name.isEmpty()==false) break;
            }
            webIdx=(((webIdx+1)<9)?(webIdx+1):(0));
        }
        iORw = !iORw;
        return  name;
    }

    void randomImage() { 
        String name = getNextImage();
        if (name.isEmpty()) {
            showRosace();
        } else {
            cleanDsp(); 
            epaperImage(name, nullptr, 0);
        }
    }
    
    void cleanDsp() {
        display->fillRect(0, 0, 212, 104, getColorBgd(invert));
    }

    void accessPointMsg() {
        cleanDsp();
        if (getProperty(WIFI)=="BAD") {
            Msg msg[] = {{fsb9, 100, 55, "Access Point"}, {fsb9, 110, 75, "is enabled..."}};
            epaperImage("/duduap.jpg", msg, 2);
        }  else {
            Msg msg[] = {{fs18, 130, 75, getProperty(HHMM).c_str()}};
            epaperImage("/gyro.jpg", msg, 1);
        }
    }

    int stopDisplay(int screen) { 
        if (screen==STOP) return STOP;
        cleanDsp();  
        display->eraseDisplay(false);
        Msg msg[] = {{fsb9, 0, 15, "W.W.Max"}, {fsb9, 130, 98, "Stopped"}}; 
        epaperImage("/rosace.jpg", msg, 2);
        epaperIcon(iconSlepping, 25, 25, false);
        display->updateWindow(0, 0, GxGDEW0213I5F_WIDTH, GxGDEW0213I5F_HEIGHT, false);
        return STOP;
    }

    int nuitDisplay(int screen) {
        if (screen==NIGHT) return NIGHT;
        cleanDsp(); 
        display->eraseDisplay(false);
        epaperImage("/matter.jpg", NULL, 0);
        epaperIcon(iconSlepping, 25, 25, false);
        display->updateWindow(0, 0, GxGDEW0213I5F_WIDTH, GxGDEW0213I5F_HEIGHT, false);
        return NIGHT;
    }

    void showRosace(){
        cleanDsp();
        if (fliphhmm) {
            Msg msg[] = {{fsb9, 5, 13, getProperty(HHMM).c_str()}, { fsb9 ,125,98, getProperty(DDMM).c_str() } }; 
            epaperImage("/rosace.jpg", msg, 2);
        } else {
            Msg msg[] = {{fsb9, 2, 13, "WATCH"}, {fsb9, 130, 98, "WINDER"}}; 
            epaperImage("/rosace.jpg", msg, 2);
        }
        fliphhmm = !fliphhmm;
    }

    void showSetup() {
        display->setTextColor(getColorTxt(invert));
        display->setFont(f6);
        display->setCursor(0, 11);
        display->printf("Version Watch Winder : %s\n", getProperty(VER).c_str());
        display->printf("Version FrameWeb     : %s\n", frame.version);
        int y = display->getCursorY();
        display->drawFastHLine(0, y - 4, GxGDEW0213I5F_HEIGHT, getColorTxt(invert));
        display->setCursor(0, y + 10);
        if (getProperty(WIFI)=="BAD") {
            display->println("Wifi: is not connected");
            display->printf("Access Pt  :   %s\n", frame.config.HostName);
            display->printf("Mac Addr   :   %s\n", getProperty(MAC).c_str());
        } else {
            display->println("Wifi: is connected");
            display->printf("Wifi IP    :   %s\n", WiFi.localIP().toString().c_str());
            display->printf("Wifi MAC   :   %s\n", WiFi.macAddress().c_str());
        }
        y = display->getCursorY();
        display->drawFastHLine(0, y - 4, GxGDEW0213I5F_HEIGHT, getColorTxt(invert));
        display->setCursor(0, y + 10);
        display->printf("Ntp server :   %s\n", Confwwm.config.ntpServer);
        display->printf("UTC time   :   %d hours\n", (int)(Confwwm.config.gmtOffset_sec / 3600));
    }

    void showHardware(){
        display->setTextColor(getColorTxt(invert));
        display->setFont(f6);
        display->setCursor(0, 11);
        display->printf("Stepper position : %s step\n", getProperty(SPOS).c_str());
        display->printf("Stepper speed    : %s step/sec.\n", getProperty(SSPE).c_str());
        display->printf("Stepper is mode  : %s\n", getProperty(SISR).c_str());
        int y = display->getCursorY();
        display->drawFastHLine(0, y - 4, GxGDEW0213I5F_HEIGHT, getColorTxt(invert));
        display->setCursor(0, y + 10);
        display->printf("One turn is  : %ld step\n", Confwwm.config.oneTurnInStep);
        display->printf("Hall edge at : %s step\n", getProperty(SCAL).c_str());
        y = display->getCursorY();
        display->drawFastHLine(0, y - 4, GxGDEW0213I5F_HEIGHT, getColorTxt(invert));
        display->setCursor(0, y + 10);
        display->printf("MQTT server : %s:%d\n", Confwwm.config.mqttServer, Confwwm.config.mqttPort);
        display->printf("MQTT user   : %s \n", Confwwm.config.mqttUser );
    }

    void clock(){
        char temp[2][5];
        cleanDsp(); 
        snprintf(temp[0], 5, "%02d", timeinfo->tm_hour);
        snprintf(temp[1], 5, "%02d", timeinfo->tm_min);
        //! OLD first font fsb24 fs18
        Msg msg[] = {{fsb24, 120, 85, temp[0]}, { fs18 ,170,60, temp[1] } }; 
        epaperImage("/clockp.jpg", msg, 2);
        int hour = timeinfo->tm_hour;
        int minute = timeinfo->tm_min;
        // adjust hour
        if (hour>12) hour-=12;
        int a = hour * 10 + minute / 6;
        drawClockHand(30, a, 120);
        drawClockHand(40, minute, 60);
    }

    void drawClockHand(int length, int value, int range) {
      double angle = calculateAngle(value, range);
      int16_t tipx = 52 + (int) round(length * cos(angle));
      int16_t tipy = 52 + (int) round(length * sin(angle));
      display->drawLine(52, 52, tipx-1 , tipy+1, getColorTxt(invert) );
      display->drawLine(52, 52, tipx ,   tipy,   getColorTxt(invert) );
      display->drawLine(52, 52, tipx+1 , tipy-1, getColorTxt(invert) );
    }

    double calculateAngle(int value, int range) {
      return 2 * PI * value / range - PI / 2;
    }
    
    int normalDisplay(int screen,  bool immediate){
        int sec = timeinfo->tm_min * 60 + timeinfo->tm_sec;
        if (immediate) sec=0;
        if (sec % modulo == 0) {
           // Serial.printf("sec=%d modulo=%d screen=%d\n\r", sec, modulo, screen);
            switch (screen) {
                case 0:  { cleanDsp(); display->eraseDisplay(false); showSetup();} break;
                case 1:  { cleanDsp(); showHardware();} break;
                case 2:  { accessPointMsg(); } break;
                case 3:  { randomImage();} break;
                case 4:  { clock(); } break;
                case 5:  { randomImage();} break;
                default: { cleanDsp(); epaperDate(); } break;
            }
            display->updateWindow(0, 0, GxGDEW0213I5F_WIDTH, GxGDEW0213I5F_HEIGHT, false);
            screen++;
            if (screen > 8) {
                screen = 2; // start screen at 2 after fisrt loop
                if (timeinfo->tm_hour > 23 && timeinfo->tm_hour < 7) {
                    modulo=60; // Flip screen at 1 minute
                } else {
                    modulo=30; // Flip screen at 30 sec
                }
            }
        }
        return screen;
    }
    
    int startDisplay() {
        //display->setTextColor(getColorTxt(invert));
        //display->fillScreen(getColorBgd(invert));
        //Msg msg[] = {{fs12, 10, 25, " Salut"},
                //!    {fsb24, 19, 87, "M"}, {fs18, 69, 92, "a"}, {fsi24, 84, 77, "X"},
        //            {fs18, 19, 87, "M"}, {fs18, 69, 92, "a"}, {fs18, 84, 77, "X"},
        //            {fs18, 117, 65, " i m e"}};
        // epaperText(msg, 5);
        display->drawBitmap(imgSTART, 0, 0, GxGDEW0213I5F_HEIGHT, GxGDEW0213I5F_WIDTH, getColorTxt(invert));
        epaperIcon(iconCalibration, 25, 25, false); 
        display->update();
        return 0;
    }

    void mainLoop(qmsgstuct dmv){
        // Display
        switch (dmv.md) {
          case start: screen=startDisplay(); break;
          case normal:  
            if (dmv.opt==night) screen=nuitDisplay(screen);     // Night detected
            else  if (dmv.opt==param) screen=normalDisplay(dmv.dbx, true); // Setting
            else screen=normalDisplay(screen, false); // Normal
            break;
          case stop: screen=stopDisplay(screen); break;
        }
        // Icon event  only one icon visible by priority Pwr LIS3DH_Click, Calib, Stepper, Wifi. Glass
        if (screen<NIGHT) { // icon inside
          String ic = getProperty(ICON);
          if (ic=="6") epaperIcon(iconBat,25, 25, true); // Priority
          else if (ic=="1") { modulo=20; epaperIcon(iconKnock, 25, 25, true); }
          else if (ic=="2") epaperIcon(iconCalibration, 25, 25, true);
          else if (ic=="3") epaperIcon(iconRotation, 25, 25, true); 
          else if (ic=="4") epaperIcon(iconWifiOff, 25, 25, true);
          else if (ic=="5") epaperIcon(iconGlass,25, 25, true);
          else  epaperIcon(NULL,25, 25, true); // no icon clean space
        }
    }

    String toString(){
        char temp[90];
        snprintf(temp, sizeof(temp), "Invert:%d Modulo:%d Screen:%d webIdx=%d imgIdx=%d imagORweb[1or0]=%d icon=%s", invert, modulo, screen, webIdx, imgIdx, iORw, getProperty(ICON).c_str());
        return String(temp);
    } 

    int getScreen(){
        return screen;
    }

  private:
    String (*getProperty)(int) = NULL;
    GxEPD_Class* display;
    struct tm* timeinfo;
    int screen = 0;
    int modulo = 20;
    bool invert = false;
    int webIdx = 0;
    int imgIdx = 0;
    bool fliphhmm = false;
    bool iORw = true;
};
#endif