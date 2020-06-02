#ifndef IMAGELOADER_H
#define IMAGELOADER_H

#include "Arduino.h"
 
#include <FS.h>
#include "SPIFFS.h" // ESP32 only

// Debug macro
#ifdef DEBUG_IMAGELOADER
  #define IlDBX(...) {Serial.print("[Il]");Serial.print(__VA_ARGS__);}
  #define IlDBXLN(...) {Serial.print("[Il]");Serial.println(__VA_ARGS__);}
  #define IlDBXMF(...) {Serial.print("[Il]");Serial.printf(__VA_ARGS__);}
#else
  #define IlDBX(...)
  #define IlDBXLN(...)
   #define IlDBXMF(...)
#endif


#define  LightnessLevel 50 // if Lightness(0.5*(max(R,G,B)+min(R,G,B))) >LightnessLevel then pixel is black

class ImageLoader {
private:
  uint8_t* bmpSpace;
  size_t   size;

public:
  ImageLoader(size_t sz);
  ~ImageLoader();
  bool load(String filename);
  uint16_t read16(fs::File &f);
  uint32_t read32(fs::File &f);
  bool renderBMP(String filename);
  void jpegInfo();
  bool renderJPEG();
  uint8_t*  clearBuffer() { memset(bmpSpace, 0, size); return bmpSpace; };
  uint8_t* getBuffer(){ return bmpSpace; };
  size_t   getSize() {return size;};
};

#endif 