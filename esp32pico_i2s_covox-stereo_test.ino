#include "driver/i2s.h"

#define STEREO_CHANNEL_SELECT 18
#define I2S_WS 19
#define I2S_SCK 22
#define I2S_SD 21
#define SAMPLE_RATE_COVOX 96000

#define VOLUME 4 // 0 min, 8 max

#define SIZE_OF_COVOX_BUF_IN_BYTES 1024*4

uint32_t buf[1024];
volatile uint32_t totalSampleCounter = 0;
volatile uint32_t totalChannelInterruptCounter = 0;
volatile uint8_t buffer_full = 0;
uint32_t totalSamplesPlayed = 0;
volatile uint8_t left = 0, right = 0;

void IRAM_ATTR isr_sample_covox_stereo() {
  uint16_t out_left = (left - 128) << VOLUME;
  uint16_t out_right = (right - 128) << VOLUME;
  uint16_t i = totalSampleCounter & 1023;
  buf[i] = (out_left << 16) | out_right;
  if (i == 511) buffer_full = 1;
  if (i == 1023) buffer_full = 2;
  totalSampleCounter++;
}

static void core0_task_covox_stereo(void *args) {
  disableCore0WDT();
  disableLoopWDT();
  while (1) {
    while (!(REG_READ(GPIO_IN_REG) & (1<<STEREO_CHANNEL_SELECT))) {}; // while fifoclk pin is low
    left = REG_READ(GPIO_IN1_REG);
    while ((REG_READ(GPIO_IN_REG) & (1<<STEREO_CHANNEL_SELECT))) {}; // while fifoclk pin is high
    right = REG_READ(GPIO_IN1_REG);
    //static uint32_t o; uint32_t n = xthal_get_ccount(); cycles = n - o; o = n; //debug //under 22 cycles
    totalChannelInterruptCounter++;
  }
}

const i2s_config_t i2s_config = {
  .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),
  .sample_rate = SAMPLE_RATE_COVOX,
  .bits_per_sample = i2s_bits_per_sample_t(16),
  .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
  .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
  .intr_alloc_flags = 0,
  .dma_buf_count = 2,
  .dma_buf_len = 1024,
  .use_apll = false
};

const i2s_pin_config_t pin_config = {
  .bck_io_num = I2S_SCK,
  .ws_io_num = I2S_WS,
  .data_out_num = I2S_SD
};

//------------------------------------------------------------------------------------------------------------------------

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

  pinMode(STEREO_CHANNEL_SELECT, INPUT); // fifoclock, 17 (Select Printer_) (PC->DSS)

  Serial.begin(115200);
  while(Serial.available());
  Serial.println(); Serial.print("--- (compilation date: "); Serial.print(__DATE__); Serial.print(" "); Serial.print(__TIME__); Serial.println(") ---");

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_start(I2S_NUM_0);

  xTaskCreatePinnedToCore(core0_task_covox_stereo, "core0_task_covox_stereo", 4096, NULL, 5, NULL, 0);
  attachInterrupt(I2S_WS, isr_sample_covox_stereo, RISING);

}

void loop() {

  if (buffer_full) {
    esp_err_t result;
    size_t bytesWritten;
    if (buffer_full == 1) {
      result = i2s_write(I2S_NUM_0, &buf[0], SIZE_OF_COVOX_BUF_IN_BYTES/2, &bytesWritten, portMAX_DELAY);
    }
    if (buffer_full == 2) {
      result = i2s_write(I2S_NUM_0, &buf[512], SIZE_OF_COVOX_BUF_IN_BYTES/2, &bytesWritten, portMAX_DELAY);
    }
    if (result != ESP_OK) Serial.println("error in i2s_write (covox)");
    totalSamplesPlayed += bytesWritten/4;
    buffer_full = 0;
  } // COVOX

  static uint32_t oldtime = 0, newtime = 0;
  newtime = micros();
  if ( (newtime-oldtime) > 100000 ) {
    //Serial.print(totalSampleCounter*4-totalSamplesPlayed); Serial.print(" / "); Serial.println(totalFifoInterruptCounter);
    //Serial.print(mode); Serial.print(" "); Serial.print(modeA); Serial.print(" "); Serial.print(modeB); Serial.print(" "); Serial.println(totalSamplesPlayed);
    Serial.println(totalChannelInterruptCounter);
    oldtime = newtime;
  }

}
