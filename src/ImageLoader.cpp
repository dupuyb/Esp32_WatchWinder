#include "ImageLoader.h"
#include <JPEGDecoder.h>  // JPEG decoder library

// this function determines the minimum of two numbers
#define minimum(a,b)     (((a) < (b)) ? (a) : (b))
#define maximum(a,b)     (((a) > (b)) ? (a) : (b))

// -------- Constructor Destructor -----------------
ImageLoader::ImageLoader(size_t sz){
  size = sz;
  bmpSpace = new uint8_t[size]; // byte array of image wxh 
  IlDBXMF("ImageLoader ok size=%d\n\r",size);
}

ImageLoader::~ImageLoader(){
	if (bmpSpace) delete[] bmpSpace;
	bmpSpace = NULL;
}

// ---------------- BMP Decoder -----------------------------
uint16_t ImageLoader::read16(fs::File &f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t ImageLoader::read32(fs::File &f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

bool ImageLoader::renderBMP(String filename) {
  bool ret = false;
  if ( !SPIFFS.exists(filename) ) {
    IlDBXLN(F("File Bmp not found")); // Can comment out if not needed
    return ret;
  }
  // Open requested file  
  fs::File bmpFS = SPIFFS.open(filename, "r");
  int32_t w, h;
  uint32_t seekOffset;
  uint16_t x,y, row, col;
  uint8_t  r, g, b;
  uint16_t code = read16(bmpFS);
  // FDBMMF("bmp code:0x%X\n\r",code);
  if (code == 0x4D42) {
    read32(bmpFS);
    read32(bmpFS);
    seekOffset = read32(bmpFS);
    read32(bmpFS);
    w = read32(bmpFS); // image width
    if (w<0) w=-w;
    h = read32(bmpFS); // image height
    if (h<0) h=-h;
    // FDBMMF("bmp seekOffset:%d w:%d h:%d\n\r", seekOffset,  w, h );
    unsigned int bit = 0;
    unsigned int iptr;
    memset(bmpSpace, 0, size);
    uint16_t frt1 = read16(bmpFS);
    uint16_t frt2 = read16(bmpFS);
    uint32_t frt3 = read32(bmpFS);
    // FDBMMF("bmp frt1:%d frt2:%d frt:%d \n\r",frt1, frt2, frt3);
    if ( (frt1 == 1) && (frt2 == 32) && (frt3 == 0) ) {
      bmpFS.seek(seekOffset);
      // Calculate padding to avoid seek
      uint16_t padding = 0; //(4 - ((w * 3) & 3)) & 3;
      uint8_t  lineBuffer[w * 4 + padding];
      int nbrByte = ceil((float)w/8.0);
      for (row = 0; row < h; row++) {
        bmpFS.read(lineBuffer, sizeof(lineBuffer));
        uint8_t*  bptr = lineBuffer;
        // http://www.apprendre-en-ligne.net/info/images/formatbmp.pdf
        // Convert 24 to 16 bit colours using the same line buffer for results
        for ( col = 0; col < w; col++) {
          x = col;
          y = row; // Display invertion or (h-1-row normal)
          b = *bptr++;
          g = *bptr++;
          r = *bptr++;
          bptr++; // bytes is not used
          // Lightness: 0.5 * (max(R,G,B) + min(R,G,B))
          // Luminosity: (0.21 * R) + (0.72 * G) + (0.07 * B)
          // Average: (R + G + B)/3
          float lu = 0.5*maximum(r,maximum(g,b))+minimum(r,minimum(g,b));
          if (lu < LightnessLevel ) {
            iptr = (x/8)+(y*nbrByte);
            bit = 7-(x % 8);
            if (iptr<size) {
               bmpSpace[iptr] |= (1<<bit);
            }
          }
        }
      }
      ret = true;
    } else { 
      IlDBXLN("BMP format not recognized.");
    }
  } else {
    IlDBXMF("BMP not compatible (0x%X)\n\r",code);
  }
  bmpFS.close();
  return ret;
}

// ----------- JPEG decoder ----------------------------------
//   Print information about the image
void ImageLoader::jpegInfo() {
  Serial.println(F("JPEG image info"));
  Serial.print(F(  "Width      :")); Serial.println(JpegDec.width);
  Serial.print(F(  "Height     :")); Serial.println(JpegDec.height);
  Serial.print(F(  "Components :")); Serial.println(JpegDec.comps);
  Serial.print(F(  "MCU / row  :")); Serial.println(JpegDec.MCUSPerRow);
  Serial.print(F(  "MCU / col  :")); Serial.println(JpegDec.MCUSPerCol);
  Serial.print(F(  "Scan type  :")); Serial.println(JpegDec.scanType);
  Serial.print(F(  "MCU width  :")); Serial.println(JpegDec.MCUWidth);
  Serial.print(F(  "MCU height :")); Serial.println(JpegDec.MCUHeight);
}

//   Decode and paint into the ePaper buffer
bool ImageLoader::renderJPEG() {
  bool ret = false;
  // retrieve infomration about the image
  uint16_t *pImg;
  uint8_t  r, g, b;
  // read each MCU block until there are no more
  int x,y,bx,by;
  unsigned int bit = 0;
  unsigned int iptr;
  memset(bmpSpace, 0, size);
  //float lumin=256;
  //float lumax=0;
  int nbrByte = ceil((float)JpegDec.width/8.0);
  while(JpegDec.read()){
    pImg = JpegDec.pImage ;
    for(by=0; by<JpegDec.MCUHeight; by++){
      for(bx=0; bx<JpegDec.MCUWidth; bx++){
        x = JpegDec.MCUx * JpegDec.MCUWidth + bx;
        y = JpegDec.MCUy * JpegDec.MCUHeight + by;
        if(x<JpegDec.width && y<JpegDec.height){
          //display.drawPixel(x,y, pImg[0]);
          // Convert 16-bit color into RGB values
          r = (uint8_t)((*pImg & 0xF800) >> 11) * 8;
          g = (uint8_t)((*pImg & 0x07E0) >> 5) * 4;
          b = (uint8_t)((*pImg & 0x001F) >> 0) * 8;
          // Lightness: 0.5 * (max(R,G,B) + min(R,G,B))
          // Luminosity: (0.21 * R) + (0.72 * G) + (0.07 * B)
          // Average: (R + G + B)/3
          float lu = 0.5*maximum(r,maximum(g,b))+minimum(r,minimum(g,b));
         // lumin=minimum(lu,lumin);
         // lumax=maximum(lu,lumax);
          if (lu < LightnessLevel ) {
            iptr = (x/8)+(y*nbrByte);
            bit = 7-(x % 8);
            if (iptr<size) {
               bmpSpace[iptr] |= (1<<bit);
            }
          }
        }
        pImg += JpegDec.comps;
      }
    }
  }
//  FDBMMF("Lu min:%f max:%f \n\r",lumin, lumax);
  // Delete data  
  JpegDec.abort();
  return ret;
}

// open file jpeg or bmp form SF file
bool ImageLoader::load(String filename) {
  bool ret = false;
  fs::File jpgFile;
  // record the current time so we can measure how long it takes to draw an image
  uint32_t drawTime = millis();
  IlDBXMF("Open:%s\n\r",filename.c_str() );
  // Note: ESP32 passes "open" test even if file does not exist, whereas ESP8266 returns NULL
  if ( !SPIFFS.exists(filename) ) {
    IlDBXMF(" Image file %s not found",filename.c_str()); // Can comment out if not needed
    ret = false;
  } else {
    if (filename.endsWith(".bmp")) {
      renderBMP (filename);
      ret = true;
    } else if (filename.endsWith(".jpg")) {
      jpgFile = SPIFFS.open(filename, "r");
      // initialise the decoder to give access to image information
      JpegDec.decodeSdFile(jpgFile);
      // print information about the image to the serial port
      // jpegInfo();
      // render the image into buffer arr
      renderJPEG();
      ret = true;
    } 
  }
  // calculate how long it took to draw the image
  drawTime = millis() - drawTime;
  // print the results to the serial port
  IlDBXMF(" Total render time was    : %d ms",drawTime);
  return ret;
}

