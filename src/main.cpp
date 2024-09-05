/********************************************************************************************* */
//    Eduboard2 ESP32-S3 Arduino Template
//    Author: Martin Burger
//    Juventus Technikerschule
//    
//    This is a example Program and by no means a complete Driver package for all the
//    Eduboard Hardware. Some Libraries are included for Example purpose.
//    Feel free to add Libraries or experiment with the existing ones by uncommenting parts
//    in the loop function.
/********************************************************************************************* */
#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include <SPIFFS.h>
#include "JpegFunc.h"
#include "focaltech.h"
#include "stk8baxx.h"
#include "RTClib.h"
#include <Encoder.h>

#define GFX_BL   -1 
#define PIN_RST  8
#define PIN_DC   16
#define PIN_CS   11
#define PIN_SCK  14
#define PIN_MOSI 13
#define PIN_MISO 12

#define PIN_ROTENC_A  41
#define PIN_ROTENC_B  40
#define PIN_ROTENC_SW 39

#define JPEG_FILENAME "/eduboard1_arduino.jpeg"

/* More data bus class: https://github.com/moononournation/Arduino_GFX/wiki/Data-Bus-Class */
Arduino_DataBus *bus = new Arduino_ESP32SPI(PIN_DC, PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO);

/* More display class: https://github.com/moononournation/Arduino_GFX/wiki/Display-Class */
//Arduino_GFX *gfx = new Arduino_ILI9488(bus, PIN_RST, 3 /* rotation */, false /* IPS */);
Arduino_GFX *gfx = new Arduino_ILI9488_18bit(bus, PIN_RST /* RST */, 1 /* rotation */, false /* IPS */);

FocalTech_Class touch;

STK8xxx stk8xxx;

RTC_PCF8563 rtc;

Encoder rotenc1(PIN_ROTENC_A, PIN_ROTENC_B);

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

static int jpegDrawCallback(JPEGDRAW *pDraw)
{
  //Serial.printf("Draw pos = %d,%d. size = %d x %d\n", pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight);
  gfx->draw16bitBeRGBBitmap(pDraw->x, pDraw->y, pDraw->pPixels, pDraw->iWidth, pDraw->iHeight);
  return 1;
}


void setup() {
  Serial.begin(115200);
  Serial.println("Eduboard2 Arduino Template");

  // Init Display
  if (!gfx->begin())
  {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(BLACK);

  if (!SPIFFS.begin())
  {
    Serial.println(F("ERROR: File System Mount Failed!"));
    gfx->println(F("ERROR: File System Mount Failed!"));
  }
  else
  {
    unsigned long start = millis();
    jpegDraw(JPEG_FILENAME, jpegDrawCallback, true /* useBigEndian */,
             0 /* x */, 0 /* y */, gfx->width() /* widthLimit */, gfx->height() /* heightLimit */);
    Serial.printf("Time used: %lu\n", millis() - start);
  }
  Wire.begin(2, 1, 400000);
  Serial.println("\nStart Touch:\n");
  Serial.printf("Touch Init: %i\n", touch.begin(Wire, FOCALTECH_SLAVE_ADDRESS));
  Serial.printf("Touch MonitorTime: %i", touch.getMonitorTime());
  touch.setPowerMode(FOCALTECH_PMODE_ACTIVE);
  touch.disableINT();

  stk8xxx.STK8xxx_Initialization();

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC is NOT initialized, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  rtc.start();

  Serial.println("End Setup\n");
}

void getGSensorData(float *X_DataOut, float *Y_DataOut, float *Z_DataOut)
{
    *X_DataOut = 0;
    *Y_DataOut = 0;
    *Z_DataOut = 0;
    stk8xxx.STK8xxx_Getregister_data(X_DataOut, Y_DataOut, Z_DataOut);
}

void rtctest() {
  DateTime now = rtc.now();

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

  Serial.print(" since midnight 1/1/1970 = ");
  Serial.print(now.unixtime());
  Serial.print("s = ");
  Serial.print(now.unixtime() / 86400L);
  Serial.println("d");

  // calculate a date which is 7 days, 12 hours and 30 seconds into the future
  DateTime future (now + TimeSpan(7,12,30,6));

  Serial.print(" now + 7d + 12h + 30m + 6s: ");
  Serial.print(future.year(), DEC);
  Serial.print('/');
  Serial.print(future.month(), DEC);
  Serial.print('/');
  Serial.print(future.day(), DEC);
  Serial.print(' ');
  Serial.print(future.hour(), DEC);
  Serial.print(':');
  Serial.print(future.minute(), DEC);
  Serial.print(':');
  Serial.print(future.second(), DEC);
  Serial.println();

  Serial.println();
}

void getTouch() {
  uint16_t x,y;
  touch.getPoint(x,y);
  uint8_t touched = touch.getTouched();  
  if(touched > 0) {    
    Serial.printf("Touched:%i\n>x:%i\n>y:%i\n", touched, x,y);
  }
}

void getGSensor() {
  float x,y,z;
  getGSensorData(&x,&y,&z);
  Serial.printf("X:%f - Y:%f - Z:%f\n", x,y,z);
}

long positionRotEnc1  = -999;
void rotEncUpdate() {
  long newEnc;
  newEnc = rotenc1.read();
  if (newEnc != positionRotEnc1) {
    Serial.print("RotEnc = ");
    Serial.print(newEnc);
    Serial.println();
    positionRotEnc1 = newEnc;
  }
}

void loop() {
  Serial.println("\n\n\n\n\n---------------------------------------------------------------\nHello Eduboard2");
  Serial.printf("Total heap: %d\n", ESP.getHeapSize());
  Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
  Serial.printf("Total PSRAM: %d\n", ESP.getPsramSize());
  Serial.printf("Free PSRAM: %d\n", ESP.getFreePsram());
  delay(1000);
  
  // getTouch();
  // delay(50);
  
  // getGSensor();
  // delay(50);

  // rtctest();
  // delay(1000);

  // rotEncUpdate();
  // delay(1);  
}
