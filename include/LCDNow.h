#ifndef LCDNOW_H
#define LCDNOW_H

#include <TFT_eSPI.h>

void initLCDNow();
void LCDNow_send(uint16_t x, uint16_t y);

#endif