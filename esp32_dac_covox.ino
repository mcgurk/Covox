#include <driver/dac.h>

volatile uint32_t totalTimerInterruptCounter = 0;

void IRAM_ATTR isr_sample() {
  uint32_t r1 = REG_READ(GPIO_IN_REG);
  uint32_t r2 = REG_READ(GPIO_IN_REG);
  uint32_t r3 = REG_READ(GPIO_IN_REG);
  uint8_t s1 = (r1 >> 16) | (r1 & B10000);
  uint8_t s2 = (r2 >> 16) | (r2 & B10000);
  uint8_t s3 = (r3 >> 16) | (r3 & B10000);
  if (s1 != s2) s1 = s3;
  dac_output_voltage(DAC_CHANNEL_2, s1);
  totalTimerInterruptCounter++;
}

static void core0_task(void *args) {
  hw_timer_t * timer = NULL;
  timer = timerBegin(0, 80, true);  // 80 (using 80 as the prescaler value), we will get a signal with a 1 MHz frequency that will increment the timer counter 1 000 000 times per second.
  timerAttachInterrupt(timer, &isr_sample, true);
  timerAlarmWrite(timer, 10, true); // 10 -> 100kHz
  timerAlarmEnable(timer);
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void setup() {
  /*Serial.begin(115200);
  while(!Serial);*/
  
  pinMode(16, INPUT); //LPT: 2 (D0)
  pinMode(17, INPUT); //     3 (D1)
  pinMode(18, INPUT); //     4 (D2)
  pinMode(19, INPUT); //     5 (D3)
  pinMode(4, INPUT); //      6 (D4)
  pinMode(21, INPUT); //     7 (D5)
  pinMode(22, INPUT); //     8 (D6)
  pinMode(23, INPUT); //     9 (D7)
                      //     GND  
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_voltage(DAC_CHANNEL_1, 0x80);
  dac_output_enable(DAC_CHANNEL_2);
  dac_output_voltage(DAC_CHANNEL_2, 0x80);

  xTaskCreatePinnedToCore(core0_task, "core0_task", 4096, NULL, 5, NULL, 0);
}

void loop() {

  /*static uint32_t old_time, new_time, old_totalTimerInterruptCounter, new_totalTimerInterruptCounter;
  new_time = millis();
  if ( (new_time-old_time) > 1000 ) {

    new_totalTimerInterruptCounter = totalTimerInterruptCounter;
    Serial.println(new_totalTimerInterruptCounter - old_totalTimerInterruptCounter); // 100100???
    old_totalTimerInterruptCounter = new_totalTimerInterruptCounter;

    old_time = new_time;
  }*/
}
