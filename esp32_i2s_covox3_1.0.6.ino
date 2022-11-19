// Arduino IDE 1.8.19, ESP32 1.0.6

#include "driver/i2s.h"

/*#define write_sample(__sample) \
  size_t __bytesWritten = 0; \
  uint8_t __s[2]; \
  __s[0] = __sample; \
  __s[1] = 0x80; \
  uint8_t __neutral = 0x80; \
  i2s_write_expand(I2S_NUM_0, &__neutral, 1, 8, 16, &__bytesWritten, portMAX_DELAY)*/

#define FIFOFULL 22
#define FIFOCLK 21

hw_timer_t * timer = NULL;
uint32_t buf[256];
volatile uint32_t totalTimerInterruptCounter = 0;
volatile uint32_t totalFifoInterruptCounter = 0;
volatile uint8_t buffer_full = 0;
volatile uint8_t front = 0;
volatile uint8_t back = 0;
volatile uint8_t fifo_buf[256];
uint32_t totalSamplesPlayed = 0;
#define fsize ((uint8_t)(back-front)) // 0-16

void IRAM_ATTR isr_fifo() {
  uint8_t s = (REG_READ(GPIO_IN_REG) >> 12); // read lpt-port state as quick as possible after interrupt is triggered
  if (fsize == 15) digitalWrite(FIFOFULL, HIGH); // buffer will be full, rise the full flag
  if (fsize < 16) fifo_buf[back++] = s; // if there is free space in buffer, put sample to buffer
  totalFifoInterruptCounter++;
}

void IRAM_ATTR onTimer() {
  uint8_t s;
  if (fsize > 0) s = fifo_buf[front++]; else s = 0x80; // if fifo is empty, play silence (128/0x80)
  //write_sample(s);
  //I2S.write(((uint16_t)s)<<8); // Right channel
  //I2S.write(0x80<<8); // Left channel // "silence"
  uint16_t i = totalTimerInterruptCounter & 255;
  //buf[i] = (s<<24) | (s<<8);
  buf[i] = (0x80<<24) | (s<<8);
  if (i == 127) buffer_full = 1;
  if (i == 255) buffer_full = 2;
  totalTimerInterruptCounter++;
  if (fsize < 16) digitalWrite(FIFOFULL, LOW); // if there is any free space in buffer, lower the full flag
}

static const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = 7000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,  // high interrupt priority
    .dma_buf_count = 2,                        // 2 buffers
    .dma_buf_len = 256,                        // Number of frames for one-time sampling. The frame here means the total data from all the channels in a WS cycle
    .use_apll = 0,
    .tx_desc_auto_clear = true,
    .fixed_mclk = -1    
};

//------------------------------------------------------------------------------------------------------------------------

void setup() {
  pinMode(12, INPUT); //LPT: 2 (D0)
  pinMode(13, INPUT); //     3 (D1)
  pinMode(14, INPUT); //     4 (D2)
  pinMode(15, INPUT); //     5 (D3)
  pinMode(16, INPUT); //     6 (D4)
  pinMode(17, INPUT); //     7 (D5)
  pinMode(18, INPUT); //     8 (D6)
  pinMode(19, INPUT); //     9 (D7)
                      //     GND
  pinMode(21, INPUT); // fifoclock, 17 (Select Printer_) (PC->DSS)
  pinMode(22, OUTPUT); digitalWrite(22, LOW); // fifofull, 10 (ACK) (DSS->PC)
  pinMode(23, OUTPUT); digitalWrite(23, HIGH); // 5V
  pinMode(32, OUTPUT); digitalWrite(32, LOW); // GND

  Serial.begin(115200);
  while(Serial.available());
  Serial.println(); Serial.print("--- (compilation date: "); Serial.print(__DATE__); Serial.print(" "); Serial.print(__TIME__); Serial.println(") ---");

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, NULL);
  //i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);

  /*uint8_t empty[256];
  for (int i=0; i< 256; i++) empty[i] = 0x80;
  size_t bytesWritten = 0;
  i2s_write_expand(I2S_NUM_0, empty, 256, 8, 16, &bytesWritten, 500 / portTICK_PERIOD_MS);
  Serial.println(bytesWritten);*/
  //i2s_write_expand(I2S_NUM_1, empty, 256, 8, 16, &bytesWritten, 500 / portTICK_PERIOD_MS);

  timer = timerBegin(0, 3, true); // 3 = prescaler
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 3808, true);
  // with prescaler 3 and count 3808 internal DAC buffer is fully syncronized with timer interrupt at 7kHz.
  timerAlarmEnable(timer);

  // "The rising edge of the pulse on Pin 17 from the printer interface is used to clock data into the FIFO"
  attachInterrupt(21, isr_fifo, RISING);
  
}


uint32_t oldtime = 0, newtime = 0;

void loop() {

  if (buffer_full) {
    size_t bytesWritten;
    if (buffer_full == 1) i2s_write(I2S_NUM_0, &buf[0], sizeof(buf)/2, &bytesWritten, portMAX_DELAY);
    if (buffer_full == 2) i2s_write(I2S_NUM_0, &buf[128], sizeof(buf)/2, &bytesWritten, portMAX_DELAY);
    totalSamplesPlayed += bytesWritten/4;
    buffer_full = 0;
    //Serial.print(front); Serial.print(" "); Serial.println(back);
  }

  newtime = micros();
  if ( (newtime-oldtime) > 50000 ) {
    Serial.print(totalTimerInterruptCounter-totalSamplesPlayed); Serial.print(" / ");
    Serial.println(totalFifoInterruptCounter);
    oldtime = newtime;
  }

}
