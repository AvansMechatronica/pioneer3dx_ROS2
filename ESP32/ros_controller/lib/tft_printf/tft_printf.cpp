#include <tft_printf.h>


Adafruit_ST7735 *printf_tft_p;

void tft_prinft_begin(Adafruit_ST7735 *printf_tft){
    printf_tft_p = printf_tft;
}

void tft_printf(int color, const char *fmt, ...){
  printf_tft_p->fillRect(0, 32, printf_tft_p->width()-1, printf_tft_p->height() - 32, ST77XX_BLACK);
  printf_tft_p->setCursor(1, 44);
  printf_tft_p->setTextColor(color);
  
  va_list args;
  va_start(args, fmt);

  while (*fmt != '\0') {
      if (*fmt == '%' && *(fmt + 1) == 'i') {
          // If the next format specifier is %d, treat the next argument as an integer
          int value = va_arg(args, int);
          printf_tft_p->printf("%i", value);
          fmt += 2; // Move to the next format specifier
      } else if (*fmt == '%' && *(fmt + 1) == 'd') {
          // If the next format specifier is %d, treat the next argument as an integer
          int value = va_arg(args, int);
          printf_tft_p->printf("%d", value);
          fmt += 2; // Move to the next format specifier
      } else if (*fmt == '%' && *(fmt + 1) == 'x') {
          // If the next format specifier is %d, treat the next argument as an hexadecimal
          int value = va_arg(args, int);
          printf_tft_p->printf("%x", value);
          fmt += 2; // Move to the next format specifier
      } else if (*fmt == '%' && *(fmt + 1) == 'f') {
          // If the next format specifier is %f, treat the next argument as a double
          double value = va_arg(args, double);
          printf_tft_p->printf("%f", value);
          fmt += 2; // Move to the next format specifier
      } else if (*fmt == '%' && *(fmt + 1) == 's') {
          // If the next format specifier is %s, treat the next argument as a string
          char *value = va_arg(args, char *);
          printf_tft_p->printf("%s", value);
          fmt += 2; // Move to the next format specifier
      } else {
          // If no special handling is needed, just print the character
          printf_tft_p->printf("%c", *fmt);
          fmt++;
      }
  }
  va_end(args);
}
