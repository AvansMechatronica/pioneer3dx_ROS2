#ifndef _TFT_PRINTF_
#define _TFT_PRINTF_

#include <Adafruit_GFX.h> // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library

void tft_prinft_begin(Adafruit_ST7735 *printf_tft);
void tft_printf(int color, const char *fmt, ...);

#endif