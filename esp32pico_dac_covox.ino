#include <driver/dac.h>

void IRAM_ATTR isr_sample() {
  dac_output_voltage(DAC_CHANNEL_2, REG_READ(GPIO_IN1_REG));
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
  pinMode(32, INPUT); //LPT: 2 (D0)
  pinMode(33, INPUT); //     3 (D1)
  pinMode(34, INPUT); //     4 (D2)
  pinMode(35, INPUT); //     5 (D3)
  pinMode(36, INPUT); //     6 (D4)
  pinMode(37, INPUT); //     7 (D5)
  pinMode(38, INPUT); //     8 (D6)
  pinMode(39, INPUT); //     9 (D7)
                      //     GND  
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_voltage(DAC_CHANNEL_1, 0x80);
  dac_output_enable(DAC_CHANNEL_2);
  dac_output_voltage(DAC_CHANNEL_2, 0x80);

  xTaskCreatePinnedToCore(core0_task, "core0_task", 4096, NULL, 5, NULL, 0);
}

void loop() {
}
