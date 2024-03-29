// Not perfect sollution for Stereo-in-1. Interrupts cannot be disabled permanently in Arduino IDE.
// Covox and DSS should work

#include "driver/i2s.h"
#include "rom/rtc.h"

#define VOLUME 4 // 0 min, 8 max
#define DEBUG
//#define EXTRA_GND 26

//COVOX
#define D0 13 // white
#define D1 14 // grey
#define D2 27 // yellow
#define D3 26 // brown
#define D4  9 // blue
#define D5 10 // purple
#define D6 18 // pink
#define D7 23 // green
#define CONVERT_GPIOREG_TO_SAMPLE(r) (uint8_t)((((r>>D0)&1)<<0) | (((r>>D1)&1)<<1) | (((r>>D2)&1)<<2) | (((r>>D3)&1)<<3) | (((r>>D4)&1)<<4) | (((r>>D5)&1)<<5) | (((r>>D6)&1)<<6) | (((r>>D7)&1)<<7))
#define STEREO_CHANNEL_SELECT 4

//DSS
#define FIFOCLK 19 // fifoclock, 17 (Select Printer_) (PC->DSS)
#define FIFOFULL 22 // fifofull, 10 (ACK) (DSS->PC)
#define SIZE_OF_DSS_BUF_IN_BYTES 256*4

// I2S:
#define I2S_WS 5
#define I2S_SCK 33
#define I2S_SD 32
#define SAMPLE_RATE_DSS 14000
#define SAMPLE_RATE_COVOX 96000

enum MODE { NONE = 0, COVOX = 1, DSS = 2, STEREO = 3 };

uint32_t buf[1024];
#define SIZE_OF_COVOX_BUF_IN_BYTES sizeof(buf)
volatile uint32_t totalSampleCounter = 0;
volatile uint32_t totalFifoInterruptCounter = 0;
volatile uint32_t totalStereoTaskCounter = 0;
volatile uint32_t buffer_full = 0;
volatile uint8_t front = 0;
volatile uint8_t back = 0;
volatile uint32_t fifo_buf[256];
MODE mode = NONE;
uint32_t totalSamplesPlayed = 0;
TaskHandle_t task_handle_dss = NULL;
TaskHandle_t task_handle_covox = NULL;
volatile uint32_t left = 0, right = 0;

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
    //totalFifoInterruptCounter++;
  }
}


// -------stereo--------

void IRAM_ATTR isr_sample_covox_stereo() {
  uint32_t l = left, r = right;
  uint16_t out_left = (CONVERT_GPIOREG_TO_SAMPLE(l)-128) << VOLUME;
  uint16_t out_right = (CONVERT_GPIOREG_TO_SAMPLE(r)-128) << VOLUME;
  uint16_t i = totalSampleCounter & 1023;
  buf[i] = (out_right << 16) | out_left;
  if (i == 511) buffer_full = 1;
  if (i == 1023) buffer_full = 2;
  totalSampleCounter++;
}

// PM7528HP: DAC A inverted, DAC B not inverted
static void core0_task_covox_stereo(void *args) {
  disableCore0WDT();
  disableLoopWDT();
  while (1) {
    //do { a = REG_READ(GPIO_IN_REG); } while (!(a&(1<<STEREO_CHANNEL_SELECT))); // a = when channel select is high
    //do { b = REG_READ(GPIO_IN_REG); } while (b&(1<<STEREO_CHANNEL_SELECT)); // b = when channel select is low
    portDISABLE_INTERRUPTS();
    uint32_t temp_reg = 0, temp_reg2 = 0, temp_reg3 = 0;
    const uint32_t gpio_reg = 0x3FF4403C, mask = (1<<STEREO_CHANNEL_SELECT);
    const uint32_t left_ptr = (uint32_t)&left;
    const uint32_t right_ptr = (uint32_t)&right;
    __asm__ __volatile__(
      "loop1: \n" 
      //"memw \n" 
      "l32i.n	%0, %3, 0 \n" // read left channel
      "bnone	%0, %4, loop1 \n" // if LOW, go back to start
      //"memw \n"
      //"l32i.n	%2, %3, 0 \n"
      //"bnone	%2, %4, loop1 \n" // if LOW, go back to start
      " \n"
      "loop2: \n"
      //"memw \n"
      "l32i.n	%1, %3, 0 \n" // read right channel
      "bany 	%1, %4, loop2 \n" // if HIGH, go back to start
      //"memw \n"
      "l32i.n	%2, %3, 0 \n"
      "bany	  %2, %4, loop2 \n" // if HIGH, go back to start
      //"memw \n"
      "s32i.n	%0, %5, 0 \n"
      //"memw \n"
      "s32i.n	%1, %6, 0 \n"
      : "=r" (temp_reg), "=r" (temp_reg2), "=r" (temp_reg3) : "a" (gpio_reg), "a" (mask), "a" (left_ptr), "a" (right_ptr) );
    portENABLE_INTERRUPTS();
    /*portDISABLE_INTERRUPTS();
    uint32_t temp_reg = 0, temp_reg2 = 0, temp_reg3 = 0;
    const uint32_t gpio_reg = 0x3FF4403C, mask = (1<<STEREO_CHANNEL_SELECT);
    const uint32_t left_ptr = (uint32_t)&left;
    const uint32_t right_ptr = (uint32_t)&right;
    __asm__ __volatile__(
      "loop2: \n"
      //"memw \n"
      "l32i.n	%1, %3, 0 \n" // read right channel
      "bany 	%1, %4, loop2 \n" // if HIGH, go back to start
      //"memw \n"
      "l32i.n	%2, %3, 0 \n"
      "bany	  %2, %4, loop2 \n" // if HIGH, go back to start
      " \n"
      "loop1: \n" 
      //"memw \n" 
      "l32i.n	%0, %3, 0 \n" // read left channel
      "bnone	%0, %4, loop1 \n" // if LOW, go back to start
      //"memw \n"
      //"l32i.n	%2, %3, 0 \n"
      //"bnone	%2, %4, loop1 \n" // if LOW, go back to start
      " \n"
      //"memw \n"
      "s32i.n	%0, %5, 0 \n"
      //"memw \n"
      "s32i.n	%1, %6, 0 \n"
      : "=r" (temp_reg), "=r" (temp_reg2), "=r" (temp_reg3) : "a" (gpio_reg), "a" (mask), "a" (left_ptr), "a" (right_ptr) );
    portENABLE_INTERRUPTS();*/
    //totalStereoTaskCounter++;
  }
 }

void IRAM_ATTR isr_channelselect() {
  Serial.println("STEREO DETECTED!!!!");
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_deep_sleep(1000); // us
}

// -----stereo ^^^^-------


const i2s_config_t i2s_config = {
  .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),
  .sample_rate = SAMPLE_RATE_COVOX,
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
      i2s_start(I2S_NUM_0);
      attachInterrupt(STEREO_CHANNEL_SELECT, isr_channelselect, RISING); // STEREO COVOX detection
      break;
    case DSS: // covox -> dss
      i2s_set_sample_rates(I2S_NUM_0, SAMPLE_RATE_DSS); // set sample rate
      i2s_start(I2S_NUM_0);
      xTaskCreatePinnedToCore(core0_task_dss, "core0_task_dss", 4096, NULL, 5, &task_handle_dss, 0); // create task_handle_dss
      attachInterrupt(I2S_WS, isr_sample_dss, RISING); // handles i2s samples
      break;
    case STEREO:
      i2s_set_sample_rates(I2S_NUM_0, SAMPLE_RATE_COVOX); // set sample rate
      i2s_start(I2S_NUM_0);
      xTaskCreatePinnedToCore(core0_task_covox_stereo, "core0_task_covox_stereo", 4096, NULL, 5, NULL, 0);
      attachInterrupt(I2S_WS, isr_sample_covox_stereo, RISING);
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
  //pinMode(EXTRA_GND, OUTPUT); digitalWrite(EXTRA_GND, LOW); // just another GND

  #ifdef DEBUG
  Serial.begin(115200);
  while(Serial.available());
  Serial.println(); Serial.print("--- (compilation date: "); Serial.print(__DATE__); Serial.print(" "); Serial.print(__TIME__); Serial.println(") ---");
  #endif

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  if (rtc_get_reset_reason(0) == 5) { // STEREO COVOX
    change_mode(STEREO);
  } else {
    change_mode(COVOX);
  }

}


void loop() {

  if ((mode == COVOX || mode == STEREO) && buffer_full) {
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
    Serial.print(millis()); Serial.print(" "); Serial.print(totalSampleCounter); Serial.print(" "); Serial.print(totalStereoTaskCounter); Serial.print(" "); Serial.println(mode);
    //Serial.print(millis()); Serial.print(" "); Serial.print(back); Serial.print(" "); Serial.print(front); Serial.print(" "); Serial.print(totalFifoInterruptCounter); Serial.print(" "); Serial.println(debug);
    oldtime += 1000000;//oldtime = newtime;
  }
  #endif

  if (mode != STEREO) { // don't autodetect COVOX or DSS in STEREO-mode
    static uint32_t dss_detect = 0; 
    if (mode == DSS) if (REG_READ(GPIO_IN_REG) & (1<<FIFOCLK)) dss_detect = millis();
    if (mode == DSS) if ((millis() - dss_detect) > 1000) change_mode(COVOX);
    //if (mode != DSS) if (REG_READ(GPIO_IN_REG) & (1<<FIFOCLK)) ESP.restart();
    if (mode != DSS) if (REG_READ(GPIO_IN_REG) & (1<<FIFOCLK)) change_mode(DSS);
  }

}
