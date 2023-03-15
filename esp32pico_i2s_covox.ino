#include <driver/i2s.h>

#define I2S_WS 19
#define I2S_SCK 22
#define I2S_SD 21
#define SAMPLE_RATE 96000

#define bufferLen 1024
int32_t sBuffer[bufferLen];

volatile uint32_t totalTimerInterruptCounter = 0;
volatile uint32_t conflictCounter = 0;
volatile uint8_t buffer_full = 0;

const i2s_config_t i2s_config = {
  .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX ),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = i2s_bits_per_sample_t(16),
  .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
  .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
  .intr_alloc_flags = 0,
  .dma_buf_count = 2,
  .dma_buf_len = bufferLen,
  .use_apll = false
};

const i2s_pin_config_t pin_config = {
  .bck_io_num = I2S_SCK,
  .ws_io_num = I2S_WS,
  .data_out_num = I2S_SD
};

void IRAM_ATTR isr_sample() {
  uint8_t s1 = REG_READ(GPIO_IN1_REG);
  uint8_t s2 = REG_READ(GPIO_IN1_REG);
  uint8_t s3 = REG_READ(GPIO_IN1_REG);
  if (s1 != s2) { s1 = s3; conflictCounter++; }
  //int16_t out = 32767-(s1 << 6);
  uint16_t out = 32767-(s1 << 6);
  uint16_t i = totalTimerInterruptCounter & 1023;
  sBuffer[i] = (out << 16) | out;
  if (i == 511) buffer_full = 1;
  if (i == 1023) buffer_full = 2;
  totalTimerInterruptCounter++;
}

static void core0_task(void *args) {
  attachInterrupt(I2S_WS, isr_sample, RISING);
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  pinMode(32, INPUT); //LPT: 2 (D0)
  pinMode(33, INPUT); //     3 (D1)
  pinMode(34, INPUT); //     4 (D2)
  pinMode(35, INPUT); //     5 (D3)
  pinMode(36, INPUT); //     6 (D4)
  pinMode(37, INPUT); //     7 (D5)
  pinMode(38, INPUT); //     8 (D6)
  pinMode(39, INPUT); //     9 (D7)
                      //     GND

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_start(I2S_NUM_0);

  xTaskCreatePinnedToCore(core0_task, "core0_task", 4096, NULL, 5, NULL, 0);
}

void loop() {

  static uint32_t byteswritten = 0, totalSamplesPlayed = 0;
  esp_err_t result;

  if (buffer_full) {
    size_t bytesWritten;
    if (buffer_full == 1) {
      result = i2s_write(I2S_NUM_0, &sBuffer[0], sizeof(sBuffer)/2, &bytesWritten, portMAX_DELAY);
    }
    if (buffer_full == 2) {
      result = i2s_write(I2S_NUM_0, &sBuffer[512], sizeof(sBuffer)/2, &bytesWritten, portMAX_DELAY);
    }
    if (result != ESP_OK) Serial.println("error in i2s_write");
    totalSamplesPlayed += bytesWritten/4;
    buffer_full = 0;
  }

  static uint32_t old_time, new_time, old_totalTimerInterruptCounter, new_totalTimerInterruptCounter;
  new_time = millis();
  if ( (new_time-old_time) > 1000 ) {
    //new_totalTimerInterruptCounter = totalTimerInterruptCounter;
    //Serial.println(new_totalTimerInterruptCounter - old_totalTimerInterruptCounter); // 100100???
    //old_totalTimerInterruptCounter = new_totalTimerInterruptCounter;
    //Serial.println(conflictCounter);
    Serial.println(totalSamplesPlayed);
    old_time = new_time;
  }
}
