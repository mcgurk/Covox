#include "driver/ledc.h"
 
ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_2_BIT,
    .timer_num  = LEDC_TIMER_0,
    .freq_hz    = 7000
};
 
ledc_channel_config_t ledc_channel = {
    .gpio_num   = 18,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .channel    = LEDC_CHANNEL_0,
    .timer_sel  = LEDC_TIMER_0,
    .duty       = 2
};

volatile uint32_t isr_count;

void IRAM_ATTR isr() {
  isr_count++;
}

void setup() {
    Serial.begin(115200);
    while(!Serial);

    ledc_timer_config(&ledc_timer);
    ledc_channel_config(&ledc_channel);

    attachInterrupt(18, isr, FALLING);
}

void loop() {

  static uint32_t old_time, new_time, old_isr_count;
  new_time = millis();
  if ( new_time > old_time ) {
    uint32_t new_isr_count = isr_count;
    Serial.println(new_isr_count-old_isr_count);
    old_time += 1000;
    old_isr_count = new_isr_count; 
  }

}
