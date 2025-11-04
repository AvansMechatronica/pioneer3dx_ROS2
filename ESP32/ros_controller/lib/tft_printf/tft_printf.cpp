#include <tft_printf.h>


Adafruit_ST7735 *printf_tft_p;

void tft_prinft_begin(Adafruit_ST7735 *printf_tft){
    printf_tft_p = printf_tft;
}

void tft_printf(int color, const char *fmt, ...) {
  char buffer[256];  // Pas grootte aan voor jouw display
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  printf_tft_p->fillRect(0, 32, printf_tft_p->width() - 1, printf_tft_p->height() - 32, ST77XX_BLACK);
  printf_tft_p->setCursor(1, 44);
  printf_tft_p->setTextColor(color);
  printf_tft_p->print(buffer);
}