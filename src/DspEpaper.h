#ifndef DspEpaper_h
#define DspEpaper_h

#include <GxEPD.h>                       // ePApaer
#include <GxGDEW0213I5F/GxGDEW0213I5F.h> // 2.13" b/w 104x212 flexible
#include "BitmapGraphics.h"
// FreeFonts from Adafruit_GFX
#include <Fonts/FreeSerif12pt7b.h>
#include <Fonts/FreeSerif18pt7b.h>
#include <Fonts/FreeSerifBold9pt7b.h>
#include <Fonts/FreeSerif9pt7b.h>
//#include <Fonts/Org_01.h>
#include <Fonts/FreeSerifBold24pt7b.h>
#include <Fonts/FreeSerifItalic24pt7b.h>
#include "Monospaced_bold_8.h"
#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>
#include "ImageLoader.h"

const GFXfont *fsi24 = &FreeSerifItalic24pt7b;
const GFXfont *fsb24 = &FreeSerifBold24pt7b;
const GFXfont *fsb9 = &FreeSerifBold9pt7b;
const GFXfont *fs9 = &FreeSerif9pt7b;
const GFXfont *f6 = &Monospaced_bold_8; //&Org_01;
const GFXfont *fs12 = &FreeSerif12pt7b;
const GFXfont *fs18 = &FreeSerif18pt7b;

// Image (bmp, jpeg) loader
ImageLoader imageLoader(GxGDEW0213I5F_WIDTH * 27); // 212/8 bytes + 104 Width

struct Msg {
  const GFXfont *font;
  int16_t x, y;
  String text;
};

#define STOP 99
typedef enum {   
  normal = 0, 
  start = 1, 
  stop = 2,
} mode;

struct dsp {
  int sec;
  mode md;
} DSP;

class DspEpaper {
public:

    DspEpaper(GxEPD_Class* dsp, AccelStepper* s, tm* tii, CalibrationWWM* c, uint8_t* lisc, int* wist, char *ver ) {
        display=dsp;
        stepper=s;
        timeinfo=tii;
        calibStp=c;
        LIS3DH_Click=lisc;
        wifiSt=wist;
        version=ver;
        modulo = 5;
    }

    // Time HH:MM
    String getTime() {
        char temp[10];
        if (timeinfo->tm_year > 100)
            snprintf(temp, 10, "%02d:%02d", timeinfo->tm_hour, timeinfo->tm_min);
        else
            strcpy(temp, "H:M");
        return String(temp);
    }

    String getDate() {
        char temp[20];
        if (timeinfo->tm_year > 100)
            snprintf(temp, 20, "%02d-%02d-%04d", timeinfo->tm_mday, (timeinfo->tm_mon + 1), (1900 + timeinfo->tm_year));
        else
            strcpy(temp, "D-M-Y");
        return String(temp);
    }

    String getMacAddress() {
        uint8_t baseMac[6];
        esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
        char baseMacChr[18] = {0};
        sprintf(baseMacChr, "%02X:%02X:%02X:%02X:%02X:%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
        return String(baseMacChr);
    }

    void epaperDate() {
        uint16_t box_x = 20;
        uint16_t box_y = 30;
        uint16_t box_w = GxGDEW0213I5F_HEIGHT - 40;
        uint16_t box_h = GxGDEW0213I5F_WIDTH - box_y;
        display->fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
        // display->drawRect(box_x, box_y+1, box_w, box_h, GxEPD_BLACK);
        display->setFont(fs18);
        display->setCursor(box_x, 55);
        display->print(getDate().c_str());
        display->setFont(fsb24);
        display->setCursor(box_x + 30, 98);
        display->print(getTime().c_str());
        display->updateWindow(box_x, box_y, box_w, box_h, true);
    }

    void epaperIcon(const uint8_t *icon, const uint8_t w, const uint8_t h, bool refresh) {
        uint16_t box_x = GxGDEW0213I5F_HEIGHT - w;
        uint16_t box_y = 0;
        uint16_t box_w = w;
        uint16_t box_h = h;
        display->fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
        display->drawBitmap(icon, box_x, box_y, box_w, box_h, GxEPD_BLACK);
        if (refresh)
            display->updateWindow(box_x, box_y, box_w, box_h, true);
    }

    void epaperText(Msg msg[], int sz) {
        for (int i = 0; i < sz; i++){
            display->setFont(msg[i].font);
            display->setCursor(msg[i].x, msg[i].y);
            display->print(msg[i].text.c_str());
        }
    }

    bool epaperImage(String filename, Msg msg[], int sz) {
        bool ret=true;
        if (SPIFFS.begin() == true) {
            // open the image file
            if (imageLoader.load(filename)) {
            display->drawBitmap(imageLoader.getBuffer(), 0, 0, GxGDEW0213I5F_HEIGHT, GxGDEW0213I5F_WIDTH, GxEPD_BLACK);
            epaperText(msg, sz);
            } else {
            ret = false;
            }
        } else {
            display->setCursor(0, 95);
            display->printf("%s error!", filename.c_str());
            ret = false;
        }
        return ret;
    }

    void accessPointMsg() {
        display->fillRect(0, 0, 212, 104, GxEPD_WHITE);
        if (*wifiSt != WL_CONNECTED) {
            Msg msg[] = {{fsb9, 100, 55, "Access Point"}, {fsb9, 110, 75, "is enabled..."}};
            epaperImage("/image3.jpg", msg, 2);
        }  else {
            Msg msg[] = {{fs18, 130, 75, getTime().c_str()}};
            epaperImage("/image2.jpg", msg, 1);
        }
    }

    int stopDisplay(int screen) {
        if (screen==STOP) return STOP;
        display->fillRect(0, 0, 212, 104, GxEPD_WHITE); 
        display->eraseDisplay(false);
        Msg msg[] = {{fsb9, 0, 13, "WATCH"}, {fsb9, 130, 100, "WINDER"}}; 
        epaperImage("/image1.jpg", msg, 2);
        epaperIcon(iconSlepping, 25, 25, false);
        display->updateWindow(0, 0, GxGDEW0213I5F_WIDTH, GxGDEW0213I5F_HEIGHT, false);
        return STOP;
    }

    void showSetup() {
        display->setTextColor(GxEPD_BLACK);
        display->setFont(f6);
        display->setCursor(0, 11);
        display->printf("Version Watch Winder : %s\n", version);
        display->printf("Version FrameWeb     : %s\n", frame.version);
        int y = display->getCursorY();
        display->drawFastHLine(0, y - 4, GxGDEW0213I5F_HEIGHT, GxEPD_BLACK);
        display->setCursor(0, y + 10);
        if (*wifiSt != WL_CONNECTED) {
            display->println("Wifi: is not connected");
            display->printf("Access Pt  :   %s\n", frame.config.HostName);
            display->printf("Mac Addr   :   %s\n", getMacAddress().c_str());
        } else {
            display->println("Wifi: is connected");
            display->printf("Wifi IP    :   %s\n", WiFi.localIP().toString().c_str());
            display->printf("Wifi MAC   :   %s\n", WiFi.macAddress().c_str());
        }
        y = display->getCursorY();
        display->drawFastHLine(0, y - 4, GxGDEW0213I5F_HEIGHT, GxEPD_BLACK);
        display->setCursor(0, y + 10);
        display->printf("Ntp server :   %s\n", Confwwm.config.ntpServer);
        display->printf("UTC time   :   %d hours\n", (int)(Confwwm.config.gmtOffset_sec / 3600));
    }

    void showHardware(){
        display->setTextColor(GxEPD_BLACK);
        display->setFont(f6);
        display->setCursor(0, 11);
        display->printf("Stepper position : %ld step\n", stepper->currentPosition());
        display->printf("Stepper speed    : %.1f step/sec.\n", stepper->speed());
        display->printf("Stepper is mode  : %s\n", ((stepper->isRunning())?("ON"):("OFF")));
        int y = display->getCursorY();
        display->drawFastHLine(0, y - 4, GxGDEW0213I5F_HEIGHT, GxEPD_BLACK);
        display->setCursor(0, y + 10);
        display->printf("One turn is  : %ld step\n", Confwwm.config.oneTurnInStep);
        display->printf("Hall edge at : %ld %ld step\n", calibStp->pIn1, calibStp->pOut1);
        y = display->getCursorY();
        display->drawFastHLine(0, y - 4, GxGDEW0213I5F_HEIGHT, GxEPD_BLACK);
        display->setCursor(0, y + 10);
        display->printf("MQTT server : %s:%d\n", Confwwm.config.mqttServer, Confwwm.config.mqttPort);
        display->printf("MQTT user   : %s \n", Confwwm.config.mqttUser );
    }

    int normalDisplay(int screen, int sec){
        if (sec % modulo == 0) {
            switch (screen) {
            case 0: { display->fillRect(0, 0, 212, 104, GxEPD_WHITE); display->eraseDisplay(false); showSetup();} break;
            case 1: break;
            case 2: { display->fillRect(0, 0, 212, 104, GxEPD_WHITE); showHardware();} break;
            case 3: break;
            case 4: { display->fillRect(0, 0, 212, 104, GxEPD_WHITE); Msg msg[] = {{fsb9, 0, 13, "WATCH"}, {fsb9, 130, 100, "WINDER"}}; epaperImage("/image1.jpg", msg, 2);} break;
            case 5: break;
            case 6: break;
            case 7: { display->fillRect(0, 0, 212, 104, GxEPD_WHITE); if (epaperImage("/web0.jpg", nullptr, 0)==false) epaperDate(); }  break;
            case 8: break;
            case 9: accessPointMsg(); break;
            default: { display->fillRect(0, 0, 212, 104, GxEPD_WHITE); epaperDate(); } break;
            }
            display->updateWindow(0, 0, GxGDEW0213I5F_WIDTH, GxGDEW0213I5F_HEIGHT, false);
            screen++;
            if (screen > 12) {
                screen = 3; // start at 3 after fisrt loop
                if (sec >= 23*3600 && sec < 6*3600) {
                    modulo=60;  // Flip screen at 1 minute
                } else {
                    modulo = 10; // Decrease loop by 2
                }
            }
        }
        return screen;
    }
    int startDisplay(int sc) {
        display->setTextColor(GxEPD_BLACK);
        display->fillScreen(GxEPD_WHITE);
        Msg msg[] = {{fs12, 10, 25, " Salut"},
                        {fsb24, 19, 87, "M"}, {fs18, 69, 92, "a"}, {fsi24, 84, 77, "X"},
                        {fs18, 117, 65, " i m e"}};
        epaperText(msg, 5);
        epaperIcon(iconCalibration, 25, 25, false); 
        display->update();
        return 0;
    }

    void mainLoop(dsp dmv){
        // Icon event  only one icon visible by priority LIS3DH_Click, Calib, Stepper, Wifi
        if (*LIS3DH_Click != 0) { modulo=5; epaperIcon(iconKnock, 25, 25, true); }
        else if (calibStp->manuCalRun || calibStp->autoCalRun) epaperIcon(iconCalibration, 25, 25, true);
        else if (calibStp->stepper->isRunning()) epaperIcon(iconRotation, 25, 25, true); 
        else if (*wifiSt != WL_CONNECTED) epaperIcon(iconWifiOff, 25, 25, true);
        else display->fillRect( 187, 0, 25, 25, GxEPD_WHITE); // no icon clean space
        // Display
        switch (dmv.md) {
          case start: screen=startDisplay(screen); break;
          case normal: screen=normalDisplay(screen, dmv.sec); break;
          case stop: screen=stopDisplay(screen); break;
        }
    }

    GxEPD_Class* display;
    struct tm* timeinfo;
    CalibrationWWM* calibStp;
    char *version;
    uint8_t* LIS3DH_Click;
    int * wifiSt;
    int screen;
    int modulo;
    AccelStepper* stepper; 
};
#endif