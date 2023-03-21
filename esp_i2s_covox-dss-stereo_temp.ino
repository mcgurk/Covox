#include "driver/i2s.h"
#include "rom/rtc.h"

#define VOLUME 4 // 0 min, 8 max
#define DEBUG
#define EXTRA_GND 26

//COVOX
#define D0 4
#define D1 13
#define D2 14
#define D3 27
#define D4 9
#define D5 10
#define D6 18
#define D7 23
#define CONVERT_GPIOREG_TO_SAMPLE(r) (uint8_t)((((r>>D0)&1)<<0) | (((r>>D1)&1)<<1) | (((r>>D2)&1)<<2) | (((r>>D3)&1)<<3) | (((r>>D4)&1)<<4) | (((r>>D5)&1)<<5) | (((r>>D6)&1)<<6) | (((r>>D7)&1)<<7))
#define STEREO_CHANNEL_SELECT 25

//DSS
#define FIFOCLK 19 // fifoclock, 17 (Select Printer_) (PC->DSS)
#define FIFOFULL 22 // fifofull, 10 (ACK) (DSS->PC)
#define SIZE_OF_DSS_BUF_IN_BYTES 256*4

// I2S:
#define I2S_WS 21 // 19
#define I2S_SCK 33 // 22
#define I2S_SD 32 //21
#define SAMPLE_RATE_DSS 14000
#define SAMPLE_RATE_COVOX 96000
//#define SAMPLE_RATE_COVOX 14000

enum MODE { NONE = 0, COVOX = 1, DSS = 2, STEREO = 3 };

uint32_t buf[1024];
#define SIZE_OF_COVOX_BUF_IN_BYTES sizeof(buf)
volatile uint32_t totalSampleCounter = 0;
volatile uint32_t totalFifoInterruptCounter = 0;
volatile uint32_t buffer_full = 0;
volatile uint8_t front = 0;
volatile uint8_t back = 0;
volatile uint32_t fifo_buf[256];
MODE mode = NONE;
uint32_t totalSamplesPlayed = 0;
TaskHandle_t task_handle_dss = NULL;
TaskHandle_t task_handle_covox = NULL;

volatile uint32_t debug = 0;

#define fcnt ((uint8_t)(back-front)) // 0-16

void IRAM_ATTR isr_sample_covox() {
  uint32_t s1 = REG_READ(GPIO_IN_REG);
  uint32_t s2 = REG_READ(GPIO_IN_REG);
  uint32_t s3 = REG_READ(GPIO_IN_REG);
  if (s1 != s2) s1 = s3;
  uint16_t out = (CONVERT_GPIOREG_TO_SAMPLE(s1) - 128) << VOLUME;
  uint16_t i = totalSampleCounter & 1023;
  buf[i] = (out << 16) | out;
  if (i == 511) buffer_full = 1;
  if (i == 1023) buffer_full = 2;
  totalSampleCounter++;
}

static void core0_task_covox(void *args) { // actually this runs in core1
  attachInterrupt(I2S_WS, isr_sample_covox, RISING);
  while (1) {
    vTaskDelay(10);
  }
}


void IRAM_ATTR isr_sample_dss() {
  static uint32_t out = 0;
  uint16_t i = totalSampleCounter & 255;
  if (i&1) {// read new "out" only every other time
    if (fcnt > 0) {
      uint32_t g = fifo_buf[front++];
      uint16_t s = (CONVERT_GPIOREG_TO_SAMPLE(g)-128) << VOLUME;
      out = (s << 16) | s;
    } else out = 0;
    if (fcnt < 16) GPIO.out_w1tc = ((uint32_t)1 << FIFOFULL); //digitalWrite(FIFOFULL, LOW);
  }
  buf[i] = out;
  if (i == 127) buffer_full = 1;
  if (i == 255) buffer_full = 2;
  totalSampleCounter++;
}

// "The rising edge of the pulse on Pin 17 from the printer interface is used to clock data into the FIFO"
// this could be optimized little more: rely on samples comes in bursts. calculate how much there is free space in fifo and fill that and then rise fifofull
// whis could be done even with unrolling while with case-command
static void core0_task_dss(void *args) {
  disableCore0WDT();
  disableLoopWDT();
  while (1) {
    while (!(REG_READ(GPIO_IN_REG) & (1<<FIFOCLK))) {}; // while fifoclk pin is low
    fifo_buf[back++] = REG_READ(GPIO_IN_REG);
    if (fcnt == 16) GPIO.out_w1ts = ((uint32_t)1 << FIFOFULL); //digitalWrite(FIFOFULL, HIGH);
    while ((REG_READ(GPIO_IN_REG) & (1<<FIFOCLK))) {}; // while fifoclk pin is high
    totalFifoInterruptCounter++;
  }
}

const i2s_config_t i2s_config = {
  .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),
  .sample_rate = SAMPLE_RATE_DSS,
  .bits_per_sample = i2s_bits_per_sample_t(16),
  .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
  .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
  .intr_alloc_flags = 0,
  .dma_buf_count = 2,
  .dma_buf_len = 512,
  .use_apll = false
};

const i2s_pin_config_t pin_config = {
  .bck_io_num = I2S_SCK,
  .ws_io_num = I2S_WS,
  .data_out_num = I2S_SD
};

void change_mode(MODE new_mode) {
  MODE old_mode = mode;
  if (new_mode == old_mode) return;
  mode = NONE;

  for (int i = 0; i < 1024; i++) buf[i] = 0;
  for (int i = 0; i < 256; i++) fifo_buf[i] = 0;
  front = 0; back = 0;
  totalSampleCounter = 0;
  buffer_full = 0;

  switch (old_mode) { //disable current mode
    case COVOX:
      vTaskDelete(task_handle_covox); //disable task
      detachInterrupt(I2S_WS); //disable i2s interrupt
      i2s_stop(I2S_NUM_0); //stop i2s   
      break;
    case DSS:
      vTaskDelete(task_handle_dss); //disable task
      detachInterrupt(I2S_WS); //disable i2s interrupt
      i2s_stop(I2S_NUM_0); //stop i2s
      break;
    default:
      break;
  }

  switch (new_mode) {
    case COVOX: //dss -> covox
      // I don't like this much... if I put COVOX to core0, DSS doesn't work after COVOX without crackling. so I must use core1 to COVOX:
      xTaskCreatePinnedToCore(core0_task_covox, "core0_task_covox", 4096, NULL, 5, &task_handle_covox, 1); // create task_handle_covox (creates I2S_WS interrupt)
      i2s_set_sample_rates(I2S_NUM_0, SAMPLE_RATE_COVOX); // set sample rate
      i2s_start(I2S_NUM_0); // start i2s_covox
      break;
    case DSS: // covox -> dss
      i2s_set_sample_rates(I2S_NUM_0, SAMPLE_RATE_DSS); // set sample rate
      i2s_start(I2S_NUM_0); // start i2s_dss
      xTaskCreatePinnedToCore(core0_task_dss, "core0_task_dss", 4096, NULL, 5, &task_handle_dss, 0); // create task_handle_dss
      attachInterrupt(I2S_WS, isr_sample_dss, RISING); // handles i2s samples
      break;
    default:
      break;
  }

  mode = new_mode;
  #ifdef DEBUG
  Serial.print("New mode!: "); Serial.println(mode);
  #endif
}

//------------------------------------------------------------------------------------------------------------------------


void setup() {
  pinMode(D0, INPUT); //LPT: 2 (D0)
  pinMode(D1, INPUT); //     3 (D1)
  pinMode(D2, INPUT); //     4 (D2)
  pinMode(D3, INPUT); //     5 (D3)
  pinMode(D4, INPUT); //     6 (D4)
  pinMode(D5, INPUT); //     7 (D5)
  pinMode(D6, INPUT); //     8 (D6)
  pinMode(D7, INPUT); //     9 (D7)
                      //     GND

  pinMode(FIFOCLK, INPUT); // fifoclock, 17 (Select Printer_) (PC->DSS)
  pinMode(FIFOFULL, OUTPUT); digitalWrite(FIFOFULL, LOW); // fifofull, 10 (ACK) (DSS->PC)
  pinMode(STEREO_CHANNEL_SELECT, INPUT_PULLUP); // LPT pin 1 (_strobe)
  pinMode(EXTRA_GND, OUTPUT); digitalWrite(EXTRA_GND, LOW); // just another GND

  #ifdef DEBUG
  Serial.begin(115200);
  while(Serial.available());
  Serial.println(); Serial.print("--- (compilation date: "); Serial.print(__DATE__); Serial.print(" "); Serial.print(__TIME__); Serial.println(") ---");
  #endif

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  change_mode(COVOX); 
  //delay(1000); 
  //change_mode(DSS);
  //if (rtc_get_reset_reason(0) == 12) change_mode(DSS); else change_mode(COVOX);
}


void loop() {

  if ((mode == COVOX) && buffer_full) {
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

  if ((mode == DSS) && buffer_full) {
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
  } // DSS

  #ifdef DEBUG
  static uint32_t oldtime = 0, newtime = 0, last = 0, oldcnt = 1111;
  newtime = micros();
  //if ( (newtime-oldtime) > 100000 ) {
  //if ( (newtime-oldtime) > 1000000 ) {
  if ( newtime > oldtime ) {
    //Serial.print(totalSampleCounter*4-totalSamplesPlayed); Serial.print(" / "); Serial.println(totalFifoInterruptCounter);
    //Serial.print(mode); Serial.print(" "); Serial.print(modeA); Serial.print(" "); Serial.print(modeB); Serial.print(" "); Serial.println(totalSamplesPlayed);
    //Serial.println(totalChannelInterruptCounter-last); last = totalChannelInterruptCounter;
    //Serial.println(millis());
    //Serial.print(millis()); Serial.print(" "); Serial.println(mode);
    Serial.print(millis()); Serial.print(" "); Serial.print(back); Serial.print(" "); Serial.print(front); Serial.print(" "); Serial.print(totalFifoInterruptCounter); Serial.print(" "); Serial.println(debug);
    oldtime += 1000000;//oldtime = newtime;
  }
  #endif

  static uint32_t dss_detect = 0; 
  if (mode == DSS) if (REG_READ(GPIO_IN_REG) & (1<<FIFOCLK)) dss_detect = millis();
  if (mode == DSS) if ((millis() - dss_detect) > 1000) change_mode(COVOX);
  //if (mode != DSS) if (REG_READ(GPIO_IN_REG) & (1<<FIFOCLK)) ESP.restart();
  if (mode != DSS) if (REG_READ(GPIO_IN_REG) & (1<<FIFOCLK)) change_mode(DSS);

}
