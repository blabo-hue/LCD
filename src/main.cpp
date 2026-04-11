

#include <TFT_eSPI.h>
#include "LCDnow.h"


TFT_eSPI tft = TFT_eSPI();

void setup() {
  Serial.begin(115200);

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  initLCDNow();
  
  Serial.println("Touch test started");
}

void loop() {

  uint16_t rx, ry;

  if (tft.getTouch(&ry, &rx)) {

    uint16_t x = ry;
    uint16_t y = rx;

  // flip Y
    x = 480 - x;


    LCDNow_send(x, y);

    tft.fillCircle(x, y, 3, TFT_RED);
  }

  delay(20);
}