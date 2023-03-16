#include "driver/i2s.h"

#define FIFOFULL 10 // fifofull, 10 (ACK) (DSS->PC)
#define FIFOCLK 9 // fifoclock, 17 (Select Printer_) (PC->DSS)
#define I2S_WS 19
#define I2S_SCK 22
#define I2S_SD 21
#define SAMPLE_RATE 14000

#define VOLUME 5 // 0 min, 8 max

uint32_t buf[256];
volatile uint32_t totalTimerInterruptCounter = 0;
volatile uint32_t totalFifoInterruptCounter = 0;
volatile uint8_t buffer_full = 0;
volatile uint8_t front = 0;
volatile uint8_t back = 0;
volatile uint8_t fifo_buf[256];

volatile uint32_t cycles;

uint32_t totalSamplesPlayed = 0;
#define fcnt ((uint8_t)(back-front)) // 0-16

// "The rising edge of the pulse on Pin 17 from the printer interface is used to clock data into the FIFO"
// this could be optimized little more: rely on samples comes in bursts. calculate how much there is free space in fifo and fill that and then rise fifofull
// whis could be done even with unrolling while with case-command
static void core0_task(void *args) {
  disableCore0WDT();
  disableLoopWDT();
  while (1) {
    while (!(REG_READ(GPIO_IN_REG) & (1<<FIFOCLK))) {}; // while fifoclk pin is low
    fifo_buf[back++] = REG_READ(GPIO_IN1_REG);
    if (fcnt == 16) GPIO.out_w1ts = ((uint32_t)1 << FIFOFULL); //digitalWrite(FIFOFULL, HIGH);
    while ((REG_READ(GPIO_IN_REG) & (1<<FIFOCLK))) {}; // while fifoclk pin is high
    //static uint32_t o; uint32_t n = xthal_get_ccount(); cycles = n - o; o = n; //debug //under 22 cycles
    //totalFifoInterruptCounter++;
  }
}

void isr_sample() {
  static uint32_t out = 0;
  uint16_t i = totalTimerInterruptCounter & 255;
  if (i&1) {// read new "out" only every other time
    if (fcnt > 0) {  
      uint16_t s = (fifo_buf[front++]-128) << VOLUME;
      out = (s << 16) | s ;
    } else out = 0;
    if (fcnt < 16) GPIO.out_w1tc = ((uint32_t)1 << FIFOFULL); //digitalWrite(FIFOFULL, LOW);
  }
  buf[i] = out;
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
  attachInterrupt(I2S_WS, isr_sample, RISING); // handles i2s samples
}


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

  static uint32_t oldtime = 0, newtime = 0;
  newtime = micros();
  if ( (newtime-oldtime) > 50000 ) {
    //Serial.print(totalTimerInterruptCounter*4-totalSamplesPlayed); Serial.print(" / "); Serial.println(totalFifoInterruptCounter);
    Serial.println(totalSamplesPlayed);
    //Serial.print(cycles); Serial.print(" "); Serial.println(totalSamplesPlayed);
    oldtime = newtime;
  }

}
