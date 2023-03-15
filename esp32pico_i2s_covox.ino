#include "driver/i2s.h"

#define FIFOFULL 10 // fifofull, 10 (ACK) (DSS->PC)
#define FIFOCLK 9 // fifoclock, 17 (Select Printer_) (PC->DSS)
#define I2S_WS 19
#define I2S_SCK 22
#define I2S_SD 21
#define SAMPLE_RATE 14000

uint32_t buf[256];
volatile uint32_t totalTimerInterruptCounter = 0;
volatile uint32_t totalFifoInterruptCounter = 0;
volatile uint8_t buffer_full = 0;
volatile uint8_t front = 0;
volatile uint8_t back = 0;
volatile uint8_t fifo_buf[256];

uint32_t totalSamplesPlayed = 0;
#define fcnt ((uint8_t)(back-front)) // 0-16

void IRAM_ATTR isr_fifo() {
  //uint8_t s1 = REG_READ(GPIO_IN1_REG);
  uint8_t s1 = REG_READ(GPIO_IN1_REG);
  uint8_t s2 = REG_READ(GPIO_IN1_REG);
  uint8_t s3 = REG_READ(GPIO_IN1_REG);
  if (s1 != s2) { s1 = s3; }
  fifo_buf[back++] = s1;
  if (fcnt == 16) digitalWrite(FIFOFULL, HIGH);
}

static void core0_task(void *args) {
  attachInterrupt(FIFOCLK, isr_fifo, RISING);
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void isr_sample() {
  uint16_t i = totalTimerInterruptCounter & 255;
  if (fcnt > 0) {  
    uint16_t s = (fifo_buf[front]-128) << 3;
    buf[i] = (s << 16) | s ;
    if (i & 1) front++; //update fifopointer only every other time
  } else buf[i] = 0;
  if (fcnt < 15) digitalWrite(FIFOFULL, LOW);
  if (i == 127) buffer_full = 1;
  if (i == 255) buffer_full = 2;
  totalTimerInterruptCounter++;
}

const i2s_config_t i2s_config = {
  .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = i2s_bits_per_sample_t(16),
  .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
  .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
  .intr_alloc_flags = 0,
  .dma_buf_count = 2,
  .dma_buf_len = 256,
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

  pinMode(FIFOCLK, INPUT); // fifoclock, 17 (Select Printer_) (PC->DSS)
  pinMode(FIFOFULL, OUTPUT); digitalWrite(FIFOFULL, LOW); // fifofull, 10 (ACK) (DSS->PC)

  Serial.begin(115200);
  while(Serial.available());
  Serial.println(); Serial.print("--- (compilation date: "); Serial.print(__DATE__); Serial.print(" "); Serial.print(__TIME__); Serial.println(") ---");

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_start(I2S_NUM_0);


  xTaskCreatePinnedToCore(core0_task, "core0_task", 4096, NULL, 5, NULL, 0);
  // "The rising edge of the pulse on Pin 17 from the printer interface is used to clock data into the FIFO"
  //attachInterrupt(FIFOCLK, isr_fifo, RISING);
  attachInterrupt(I2S_WS, isr_sample, RISING);
  //attachInterrupt(I2S_WS, isr_sample, FALLING);
  
}


uint32_t oldtime = 0, newtime = 0;

void loop() {

  if (buffer_full) {
    size_t bytesWritten;
    if (buffer_full == 1) {
      i2s_write(I2S_NUM_0, &buf[0], sizeof(buf)/2, &bytesWritten, portMAX_DELAY);
    }
    if (buffer_full == 2) {
      i2s_write(I2S_NUM_0, &buf[128], sizeof(buf)/2, &bytesWritten, portMAX_DELAY);
    }
    totalSamplesPlayed += bytesWritten/4;
    buffer_full = 0;
  }

  newtime = micros();
  if ( (newtime-oldtime) > 50000 ) {
    //Serial.print(totalTimerInterruptCounter*4-totalSamplesPlayed); Serial.print(" / "); Serial.println(totalFifoInterruptCounter);
    Serial.println(totalSamplesPlayed);
    oldtime = newtime;
  }

}
