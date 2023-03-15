#include "AudioTools.h"

#define I2S_WS 19
#define I2S_SCK 22
#define I2S_SD 21
#define SAMPLE_RATE 96000
//#define SAMPLE_RATE 32000

I2SStream i2s; // final output of decoded stream
VolumeStream volume(i2s);

#define bufferLen 1024
int32_t sBuffer[bufferLen];

volatile uint32_t totalTimerInterruptCounter = 0;
volatile uint32_t conflictCounter = 0;
volatile uint8_t buffer_full = 0;

void IRAM_ATTR isr_sample() {
  uint8_t s1 = REG_READ(GPIO_IN1_REG);
  uint8_t s2 = REG_READ(GPIO_IN1_REG);
  uint8_t s3 = REG_READ(GPIO_IN1_REG);
  if (s1 != s2) { s1 = s3; conflictCounter++; }
  //int16_t out = 32767-(s1 << 6);
  //uint16_t out = 32767-(s1 << 6);
  uint16_t out = 32767-(s1 << 8);
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

  // setup i2s
  auto config = i2s.defaultConfig(TX_MODE);
  config.sample_rate = SAMPLE_RATE;
  config.channels = 2;
  config.bits_per_sample = sizeof(int16_t)*8; 
  config.pin_ws = 19;
  config.pin_bck = 22;
  config.pin_data = 21;
  i2s.begin(config);

  volume.begin(config); // we need to provide the bits_per_sample and channels
  volume.setVolume(0.3);

  xTaskCreatePinnedToCore(core0_task, "core0_task", 4096, NULL, 5, NULL, 0);
}

void loop() {

  static uint32_t byteswritten = 0, totalSamplesPlayed = 0;
  esp_err_t result;

  if (buffer_full) {
    size_t bytesWritten;
    if (buffer_full == 1) {
      //result = i2s_write(I2S_NUM_0, &sBuffer[0], sizeof(sBuffer)/2, &bytesWritten, portMAX_DELAY);
      //size_t bytesWritten = i2s.write((uint8_t*)&sBuffer[0], sizeof(sBuffer)/2);
      size_t bytesWritten = volume.write((uint8_t*)&sBuffer[0], sizeof(sBuffer)/2);
    }
    if (buffer_full == 2) {
      //result = i2s_write(I2S_NUM_0, &sBuffer[512], sizeof(sBuffer)/2, &bytesWritten, portMAX_DELAY);
      //size_t bytesWritten = i2s.write((uint8_t*)&sBuffer[512], sizeof(sBuffer)/2);
      size_t bytesWritten = volume.write((uint8_t*)&sBuffer[512], sizeof(sBuffer)/2);
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
