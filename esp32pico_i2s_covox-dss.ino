#include "driver/i2s.h"

#define FIFOCLK 9 // fifoclock, 17 (Select Printer_) (PC->DSS)
#define FIFOFULL 10 // fifofull, 10 (ACK) (DSS->PC)
#define I2S_WS 19
#define I2S_SCK 22
#define I2S_SD 21
#define SAMPLE_RATE_DSS 14000
#define SAMPLE_RATE_COVOX 96000

#define VOLUME 5 // 0 min, 8 max

#define SIZE_OF_DSS_BUF_IN_BYTES 256*4
#define SIZE_OF_COVOX_BUF_IN_BYTES 1024*4

uint32_t buf[1024];
volatile uint32_t totalTimerInterruptCounter = 0;
volatile uint32_t totalFifoInterruptCounter = 0;
volatile uint8_t buffer_full = 0;
volatile uint8_t front = 0;
volatile uint8_t back = 0;
volatile uint8_t fifo_buf[256];
volatile uint8_t modeA = 0;
volatile uint8_t modeB = 0;

volatile uint32_t cycles;

uint32_t totalSamplesPlayed = 0;
#define fcnt ((uint8_t)(back-front)) // 0-16

void IRAM_ATTR isr_sample_covox() {
  modeA = 1; // DEBUG!!!
  uint8_t s1 = REG_READ(GPIO_IN1_REG);
  uint8_t s2 = REG_READ(GPIO_IN1_REG);
  uint8_t s3 = REG_READ(GPIO_IN1_REG);
  //if (s1 != s2) { s1 = s3; conflictCounter++; }
  if (s1 != s2) s1 = s3;
  uint16_t out = (s1 - 128) << VOLUME;
  uint16_t i = totalTimerInterruptCounter & 1023;
  buf[i] = (out << 16) | out;
  if (i == 511) buffer_full = 1;
  if (i == 1023) buffer_full = 2;
  totalTimerInterruptCounter++;
}

void IRAM_ATTR isr_sample_dss() {
  modeA = 2; // DEBUG!!!
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

static void core0_task_covox(void *args) {
  attachInterrupt(I2S_WS, isr_sample_covox, RISING);
  while (1) {
    modeB = 1; // DEBUG!!!
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// "The rising edge of the pulse on Pin 17 from the printer interface is used to clock data into the FIFO"
// this could be optimized little more: rely on samples comes in bursts. calculate how much there is free space in fifo and fill that and then rise fifofull
// whis could be done even with unrolling while with case-command
static void core0_task_dss(void *args) {
  disableCore0WDT();
  disableLoopWDT();
  while (1) {
    modeB = 2; // DEBUG!!!
    while (!(REG_READ(GPIO_IN_REG) & (1<<FIFOCLK))) {}; // while fifoclk pin is low
    fifo_buf[back++] = REG_READ(GPIO_IN1_REG);
    if (fcnt == 16) GPIO.out_w1ts = ((uint32_t)1 << FIFOFULL); //digitalWrite(FIFOFULL, HIGH);
    while ((REG_READ(GPIO_IN_REG) & (1<<FIFOCLK))) {}; // while fifoclk pin is high
    //static uint32_t o; uint32_t n = xthal_get_ccount(); cycles = n - o; o = n; //debug //under 22 cycles
    //totalFifoInterruptCounter++;
  }
}


const i2s_config_t i2s_config_dss = {
  .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),
  .sample_rate = SAMPLE_RATE_DSS,
  .bits_per_sample = i2s_bits_per_sample_t(16),
  .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
  .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
  .intr_alloc_flags = 0,
  .dma_buf_count = 2,
  .dma_buf_len = 256,
  .use_apll = false
};

const i2s_config_t i2s_config_covox = {
  .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX ),
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

TaskHandle_t task_handle_dss = NULL;
TaskHandle_t task_handle_covox = NULL;

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

  i2s_driver_install(I2S_NUM_0, &i2s_config_dss, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_start(I2S_NUM_0);

  xTaskCreatePinnedToCore(core0_task_dss, "core0_task_dss", 4096, NULL, 5, &task_handle_dss, 0);
  attachInterrupt(I2S_WS, isr_sample_dss, RISING); // handles i2s samples

  jotain();

}

void jotain() {
  //clear buf?
  
  //dss -> covox
  //if fifoclock low 0,5s?
  //stop i2swrite loop
  detachInterrupt(I2S_WS); //disable i2s_dss i2s interrupt
  i2s_stop(I2S_NUM_0); //stop i2s_dss
  vTaskDelete(task_handle_dss); //disable task_handle_dss
  xTaskCreatePinnedToCore(core0_task_covox, "core0_task_covox", 4096, NULL, 5, &task_handle_covox, 0); //create task_handle_covox (creates I2S_WS interrupt)
  //i2s_driver_install(I2S_NUM_0, &i2s_config_covox, 0, NULL);
  //i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_set_sample_rates(I2S_NUM_0, SAMPLE_RATE_COVOX); //set sample rates
  i2s_start(I2S_NUM_0); //create i2s_covox
  //start i2swrite loop
  
  //covox->dss
  //if fifoclock rising
  //stop i2swrite loop
  /*detachInterrupt(I2S_WS); //disable i2s_covox i2s interrupt
  //reset fifopointers? (back front)
  i2s_stop(I2S_NUM_0); //Stop i2s_covox
  vTaskDelete(task_handle_covox); //disable task_handle_covox
  xTaskCreatePinnedToCore(core0_task_dss, "core0_task_dss", 4096, NULL, 5, NULL, &task_handle_dss, 0); //create task_handle_dss
  attachInterrupt(I2S_WS, isr_sample_dss, RISING); // handles i2s samples
  i2s_set_sample_rates(I2S_NUM_0, SAMPLE_RATE_DSS); //set sample rates
  i2s_start(I2S_NUM_0); //create i2s_dss*/
  //start i2swrite loop
  
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

  /*if (buffer_full) {
    esp_err_t result;
    size_t bytesWritten;
    if (buffer_full == 1) {
      result = i2s_write(I2S_NUM_0, &buf[0], SIZE_OF_DSS_BUF_IN_BYTES/2, &bytesWritten, portMAX_DELAY);
    }
    if (buffer_full == 2) {
      result = i2s_write(I2S_NUM_0, &buf[128], SIZE_OF_DSS_BUF_IN_BYTES/2, &bytesWritten, portMAX_DELAY);
    }
    if (result != ESP_OK) Serial.println("error in i2s_write (dss)");
    totalSamplesPlayed += bytesWritten/4;
    buffer_full = 0;
  }*/ // DSS

  static uint32_t oldtime = 0, newtime = 0;
  newtime = micros();
  if ( (newtime-oldtime) > 50000 ) {
    //Serial.print(totalTimerInterruptCounter*4-totalSamplesPlayed); Serial.print(" / "); Serial.println(totalFifoInterruptCounter);
    Serial.print(modeA); Serial.print(" "); Serial.print(modeB); Serial.print(" "); Serial.println(totalSamplesPlayed);
    //Serial.print(cycles); Serial.print(" "); Serial.println(totalSamplesPlayed);
    oldtime = newtime;
  }

}
