// ESP32 internal dac out: Pin 25 / GND (500ohm, 10uF -> OUT+ / 500ohm, 0,1uF -> GND)

#include "soc/rtc_wdt.h"

void setup() {
  pinMode(16, INPUT); //LPT: 2 (D0)
  pinMode(17, INPUT); //     3 (D1)
  pinMode(18, INPUT); //     4 (D2)
  pinMode(19, INPUT); //     5 (D3)
  pinMode(4, INPUT); //      6 (D4)
  pinMode(21, INPUT); //     7 (D5)
  pinMode(22, INPUT); //     8 (D6)
  pinMode(23, INPUT); //     9 (D7)
                      //     GND

  rtc_wdt_protect_off();
  rtc_wdt_disable();
}

void loop() {
  noInterrupts();
  while (1) {
    int32_t gpio = REG_READ(GPIO_IN_REG);
    uint8_t value = (gpio >> 12);
    dacWrite(25, value); // ~34us? why so slow? output rate is only about 30kHz...
  }
}
