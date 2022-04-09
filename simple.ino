#include "soc/rtc_wdt.h"


void setup()
{
  pinMode(12, INPUT);
  pinMode(13, INPUT);
  pinMode(14, INPUT);
  pinMode(15, INPUT);
  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pinMode(18, INPUT);
  pinMode(19, INPUT);

  rtc_wdt_protect_off();
  rtc_wdt_disable();
}

void loop()
{
  noInterrupts();
  while (1) {
    int32_t gpio = REG_READ(GPIO_IN_REG);
    uint8_t value = (gpio >> 12);
    dacWrite(25, value);
  }
}
