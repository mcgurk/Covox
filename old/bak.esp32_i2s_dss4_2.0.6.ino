// Arduino IDE 2.0.3, ESP32 2.0.6 (based ESP-IDF 4.4.3)

/*#include "soc/rtc_wdt.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"*/

#define FIFOFULL 32 // fifofull, 10 (ACK) (DSS->PC)
#define FIFOCLK 35 // fifoclock, 17 (Select Printer_) (PC->DSS)

hw_timer_t * timer = NULL;
volatile uint8_t buf[256];
volatile uint8_t front = 0;
volatile uint8_t back = 0;
volatile uint32_t samples_played = 0;
volatile uint32_t samples_not_played = 0;
volatile uint32_t totalFifoInterruptCounter = 0;

#define fcnt ((uint8_t)(back-front)) // 0-16

void IRAM_ATTR onTimer() {
  if (fcnt == 0) { // fifo empty
    samples_not_played++;
    return;
  }
  uint8_t s = buf[front++]; // koska sampleja fifossa, soita sample ja päivitä pointteri
  digitalWrite(FIFOFULL, LOW); // there is now at least one place free in buffer, lower the full flag
  //uint32_t value = (0x80 << 24) | (s << 8);
  dacWrite(26, s);
  samples_played++;
}

/*static void dac_output_task(void *args)
{
  while (1) {
    const TickType_t xDelay = 240000000/7000;
    vTaskDelay(xDelay); //240 000 000 / 7 000 = 34 285,714
    if (fcnt == 0) { // fifo empty
      samples_not_played++;
      continue;
    }
    uint8_t s = buf[front++]; // koska sampleja fifossa, soita sample ja päivitä pointteri
    digitalWrite(FIFOFULL, LOW); // there is now at least one place free in buffer, lower the full flag
    uint32_t value = (0x80 << 24) | (s << 8);
    dacWrite(26, value);
    
    //dacWrite(26, value);
    samples_played++;
    //vTaskDelay(pdMS_TO_TICKS(500));
    //const TickType_t xDelay = 240000000/7000;
    //vTaskDelay(xDelay); //240 000 000 / 7 000 = 34 285,714
    //vTaskDelay(34286); //240 000 000 / 7 000 = 34 285,714
    //ei delayta jos samplea ei soitettu?

  }
}*/

void IRAM_ATTR isr_fifo() {
  // tulee sample
  totalFifoInterruptCounter++;
  if (fcnt == 16) return;
  uint32_t r = REG_READ(GPIO_IN_REG);
  uint8_t s = (r >> 16) | (r & B10000);
  buf[back++] = s; // if there is free space in buffer, put sample to buffer
  if (fcnt == 16) {
    digitalWrite(FIFOFULL, HIGH); // buffer full, rise the full flag
    //return;
  }
}


//------------------------------------------------------------------------------------------------------------------------

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
  pinMode(FIFOCLK, INPUT); // fifoclock, 17 (Select Printer_) (PC->DSS)
  pinMode(FIFOFULL, OUTPUT); digitalWrite(FIFOFULL, LOW); // fifofull, 10 (ACK) (DSS->PC)

  Serial.begin(115200);
  while(Serial.available());
  Serial.println(); Serial.print("--- (compilation date: "); Serial.print(__DATE__); Serial.print(" "); Serial.print(__TIME__); Serial.println(") ---");

  //uint32_t t1 = portGET_RUN_TIME_COUNTER_VALUE();
  dacWrite(25, 0x80); dacWrite(26, 0x80);

  //xTaskCreatePinnedToCore(dac_output_task, "dac_chan0_output_task", 4096, NULL, 5, NULL, 0);
  //Serial.print("setup() running on core "); Serial.println(xPortGetCoreID());

  timer = timerBegin(0, 3, true); // 3 = prescaler
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 3808, true);
  // with prescaler 3 and count 3808 internal DAC buffer is fully syncronized with timer interrupt at 7kHz.
  timerAlarmEnable(timer);

  // "The rising edge of the pulse on Pin 17 from the printer interface is used to clock data into the FIFO"
  attachInterrupt(FIFOCLK, isr_fifo, RISING);
  
}


uint32_t oldtime = 0, newtime = 0;

void loop() {

  /*newtime = micros();
  if ( (newtime-oldtime) > 1000000 ) {
    //Serial.print(totalTimerInterruptCounter-totalSamplesPlayed); Serial.print(" / ");
    Serial.print(totalFifoInterruptCounter); Serial.print(" / "); Serial.println(samples_played);
    oldtime = newtime;
  }*/

  Serial.print(totalFifoInterruptCounter); Serial.print(" / "); Serial.print(samples_played); Serial.print(" / "); Serial.println(samples_not_played);
  for (int i=0; i<256; i++) { Serial.print(buf[i]); Serial.print(" "); } Serial.println();
  delay(1000);

}
