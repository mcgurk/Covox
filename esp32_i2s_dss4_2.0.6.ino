// Arduino IDE 2.0.3, ESP32 2.0.6 (based on ESP-IDF 4.4.3)

#include "driver/i2s.h"

#define FIFOFULL 32 // fifofull, 10 (ACK) (DSS->PC)
#define FIFOCLK 35 // fifoclock, 17 (Select Printer_) (PC->DSS)

hw_timer_t * timer = NULL;
volatile uint8_t buf[256];
uint32_t outbuf[512];
volatile uint32_t totalTimerInterruptCounter = 0;
volatile uint32_t totalFifoInterruptCounter = 0;
volatile uint8_t buffer_full = 0;
volatile uint8_t front = 0;
volatile uint8_t back = 0;
volatile uint8_t fifo_buf[256];

uint32_t totalSamplesPlayed = 0;
#define fcnt ((uint8_t)(back-front)) // 0-16

void IRAM_ATTR isr_fifo() {
  if (fcnt == 15) digitalWrite(FIFOFULL, HIGH); // buffer will be full, rise the full flag
  totalFifoInterruptCounter++;
  if (fcnt == 16) return; // if full, return
  uint32_t r = REG_READ(GPIO_IN_REG);
  uint8_t s = (r >> 16) | (r & B10000);
  fifo_buf[back++] = s; // put sample to buffer
}

static void core0_task(void *args) {
  attachInterrupt(FIFOCLK, isr_fifo, RISING);
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

/*void IRAM_ATTR isr_fifo() {
  totalFifoInterruptCounter++;
  if (fcnt == 16) return; // if full, return
  uint32_t r = REG_READ(GPIO_IN_REG);
  uint8_t s = (r >> 16) | (r & B10000);
  fifo_buf[back++] = s; // put sample to buffer
  if (fcnt == 16) digitalWrite(FIFOFULL, HIGH); // buffer is full, rise the full flag
}*/

void IRAM_ATTR onTimer() {
  uint8_t s;
  if (fcnt == 0) s = 0x80; else s = fifo_buf[front++]; // if fifo is empty, play silence (128/0x80)
  digitalWrite(FIFOFULL, LOW); // there must be at least one free slot in fifo at this point
  uint16_t i = totalTimerInterruptCounter & 255;
  buf[i] = s;
  if (i == 127) buffer_full = 1;
  if (i == 255) buffer_full = 2;
  totalTimerInterruptCounter++;
}

static const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = 28000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_LSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,  // high interrupt priority
    .dma_buf_count = 2,                        // 2 buffers
    .dma_buf_len = 256,                        // Number of frames for one-time sampling. The frame here means the total data from all the channels in a WS cycle
    .use_apll = 0,
    .tx_desc_auto_clear = true,
    .fixed_mclk = -1
};

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

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);

  timer = timerBegin(0, 4, true); // 4 = prescaler
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 2856, true); //sampleja tulee ihan hitusen enemmän kuin mitä i2s ehtii syödä // 2857   4   7000.350017500875
  timerAlarmEnable(timer);

  xTaskCreatePinnedToCore(core0_task, "core0_task", 4096, NULL, 5, NULL, 0);
  // "The rising edge of the pulse on Pin 17 from the printer interface is used to clock data into the FIFO"
  //attachInterrupt(FIFOCLK, isr_fifo, RISING);
  
}


uint32_t oldtime = 0, newtime = 0;

void loop() {

  if (buffer_full) {
    size_t bytesWritten;
    if (buffer_full == 1) {
      for (int i=0; i<128; i++) { uint32_t s = (0x80<<24) | (buf[i]<<8); outbuf[i*4] = s; outbuf[i*4+1] = s; outbuf[i*4+2] = s; outbuf[i*4+3] = s; }
      //i2s_write(I2S_NUM_0, &outbuf[0], sizeof(outbuf), &bytesWritten, portMAX_DELAY);
      i2s_write(I2S_NUM_0, &outbuf[0], sizeof(outbuf), &bytesWritten, 10);
    }
    if (buffer_full == 2) {
      for (int i=0; i<128; i++) { uint32_t s = (0x80<<24) | (buf[i+128]<<8); outbuf[i*4] = s; outbuf[i*4+1] = s; outbuf[i*4+2] = s; outbuf[i*4+3] = s; }
      //i2s_write(I2S_NUM_0, &outbuf[0], sizeof(outbuf), &bytesWritten, portMAX_DELAY);
      i2s_write(I2S_NUM_0, &outbuf[0], sizeof(outbuf), &bytesWritten, 10);
    }
    totalSamplesPlayed += bytesWritten/4;
    buffer_full = 0;
  }

  /*Serial.print(totalFifoInterruptCounter); Serial.print(" / "); Serial.print(samples_played); Serial.print(" / "); Serial.println(samples_not_played);
  for (int i=0; i<256; i++) { Serial.print(buf[i]); Serial.print(" "); } Serial.println();
  delay(1000);*/

  newtime = micros();
  if ( (newtime-oldtime) > 50000 ) {
    Serial.print(totalTimerInterruptCounter*4-totalSamplesPlayed); Serial.print(" / "); Serial.println(totalFifoInterruptCounter);
    oldtime = newtime;
  }

}
