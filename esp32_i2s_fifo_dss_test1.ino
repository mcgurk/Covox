#include <driver/i2s.h>
#include "driver/ledc.h"

#define SAMPLE_RATE 28000

#define BUFFER_SIZE 1024
int32_t I2S_buffer[BUFFER_SIZE*2];
const int32_t* buf1 = &I2S_buffer[0];
const int32_t* buf2 = &I2S_buffer[BUFFER_SIZE];
#define BUFFER_SIZE_IN_BYTES BUFFER_SIZE*4
volatile uint8_t buffer_full = 0;
volatile uint32_t samples_in = 0;
volatile uint32_t isr_count = 0;

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

static const i2s_config_t i2s_config_dac = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_LSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,  // high interrupt priority
    .dma_buf_count = 2,                        // 2 buffers
    .dma_buf_len = BUFFER_SIZE,                // Number of frames for one-time sampling. The frame here means the total data from all the channels in a WS cycle
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = -1
};

volatile uint32_t wscount;

void IRAM_ATTR isr_sample() {
  static int32_t buffer_ptr = 0;
  //read sample
  //uint32_t r = REG_READ(GPIO_IN_REG);
  //uint8_t s = (r >> 16) | (r & B10000);
  //uint32_t out = (0x80<<24) | (s<<8); 
  uint32_t out = 0;
  I2S_buffer[buffer_ptr++] = out;
  I2S_buffer[buffer_ptr++] = out;
  I2S_buffer[buffer_ptr++] = out;
  I2S_buffer[buffer_ptr++] = out;
  buffer_ptr = buffer_ptr & 1023;
  //buffer_ptr = (buffer_ptr + 1) & 1023;
  if (buffer_ptr == BUFFER_SIZE) buffer_full = 1;
  if (buffer_ptr == 0) buffer_full = 2;
  //samples_in++;
  samples_in += 4;
  isr_count++;
}

void setup() {
  /*
  pinMode(16, INPUT); //LPT: 2 (D0)
  pinMode(17, INPUT); //     3 (D1)
  pinMode(18, INPUT); //     4 (D2)
  pinMode(19, INPUT); //     5 (D3)
  pinMode(4, INPUT); //      6 (D4)
  pinMode(21, INPUT); //     7 (D5)
  pinMode(22, INPUT); //     8 (D6)
  pinMode(23, INPUT); //     9 (D7)
  */
  Serial.begin(115200);
  while(!Serial);

  i2s_driver_install(I2S_NUM_0, &i2s_config_dac, 0, NULL);
  i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
  
  ledc_timer_config(&ledc_timer);
  ledc_channel_config(&ledc_channel);

  attachInterrupt(18, isr_sample, RISING);

}

void loop() {

  size_t bytesOut;
  esp_err_t result;
  static uint32_t byteswritten = 0;
  
  if (buffer_full) {
    if (buffer_full == 1) result = i2s_write(I2S_NUM_0, buf1, BUFFER_SIZE_IN_BYTES, &bytesOut, portMAX_DELAY);
    if (buffer_full == 2) result = i2s_write(I2S_NUM_0, buf2, BUFFER_SIZE_IN_BYTES, &bytesOut, portMAX_DELAY);
    if (result != ESP_OK) Serial.println("error in i2s_write");
    buffer_full = 0;
    byteswritten += bytesOut;
  }
  
  static uint32_t old_time, new_time, old_isr_count;
  new_time = millis();
  if ( new_time > old_time ) {
    uint32_t new_isr_count = isr_count;
    Serial.println(new_isr_count - old_isr_count);
    old_isr_count = new_isr_count;
    Serial.println(samples_in-(byteswritten/4));
    old_time += 1000;
  }
  //delay(10);
}
