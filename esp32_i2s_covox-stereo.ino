#include "driver/i2s.h"
#include "rom/rtc.h"

#define STEREO_CHANNEL_SELECT 25
#define I2S_WS 19
#define I2S_SCK 22
#define I2S_SD 21
#define SAMPLE_RATE_COVOX 96000

#define D0 4
#define D1 13
#define D2 14
#define D3 27
#define D4 9
#define D5 10
#define D6 18
#define D7 23

#define CONVERT_GPIOREG_TO_SAMPLE(r) (uint8_t)((((r>>D0)&1)<<0) | (((r>>D1)&1)<<1) | (((r>>D2)&1)<<2) | (((r>>D3)&1)<<3) | (((r>>D4)&1)<<4) | (((r>>D5)&1)<<5) | (((r>>D6)&1)<<6) | (((r>>D7)&1)<<7))

#define VOLUME 4 // 0 min, 8 max

#define SIZE_OF_COVOX_BUF_IN_BYTES 2048*4

uint32_t buf[2048];
volatile uint32_t totalSampleCounter = 0;
volatile uint32_t totalChannelInterruptCounter = 0;
volatile uint8_t buffer_full = 0;
uint32_t totalSamplesPlayed = 0;
volatile uint32_t left = 0, right = 0;
volatile uint32_t cycles = 0;

void IRAM_ATTR isr_sample_covox_stereo() {
  //uint16_t out_left = (left - 128) << VOLUME;
  //uint16_t out_right = (right - 128) << VOLUME;
  uint32_t l = left, r = right;
  uint16_t out_left = (CONVERT_GPIOREG_TO_SAMPLE(l)-128) << VOLUME;
  uint16_t out_right = (CONVERT_GPIOREG_TO_SAMPLE(r)-128) << VOLUME;
  //uint16_t out_left = 0, out_right = 0;
  uint16_t i = totalSampleCounter & 2047;
  buf[i] = (out_right << 16) | out_left;
  if (i == 1023) buffer_full = 1;
  if (i == 2047) buffer_full = 2;
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
    uint32_t temp_reg;
    const uint32_t i1 = 0x3FF4403C, i2 = (1<<STEREO_CHANNEL_SELECT);
    __asm__ __volatile__(
      //"movi %0, 0x3FF4403C \n" // GPIO_IN_REG
      //"movi %1, 0x02000000 \n" // 1 << 25
      "loop1: \n" 
      //"memw \n" 
      "l32i.n	%0, %3, 0 \n" // read left channel
      "bnone	%0, %4, loop1 \n" // if LOW, go back to start
      //"memw \n"
      "l32i.n	%2, %3, 0 \n"
      "bnone	%2, %4, loop1 \n" // if LOW, go back to start
      " \n"
      "loop2: \n"
      //"memw \n"
      "l32i.n	%1, %3, 0 \n" // read right channel
      "bany 	%1, %4, loop2 \n" // if HIGH, go back to start
      //"memw \n"
      "l32i.n	%2, %3, 0 \n"
      "bany	  %2, %4, loop2 \n" // if HIGH, go back to start
      //: "=a" (left), "=a" (right), "=r" (r3) : "i" (0x3FF4403C), "i" (0x02000000));
      : "=a" (left), "=a" (right), "=r" (temp_reg) : "a" (i1), "a" (i2));
      //: "=r" (r1), "=r" (r2), "=a" (left), "=a" (right), "=r" (r3));
    portENABLE_INTERRUPTS();
  }
 }

void IRAM_ATTR isr_channelselect() {
  //right = left = REG_READ(GPIO_IN_REG);
  //totalChannelInterruptCounter++;
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_deep_sleep(1000); // us
}

const i2s_config_t i2s_config = {
  .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),
  .sample_rate = SAMPLE_RATE_COVOX,
  .bits_per_sample = i2s_bits_per_sample_t(16),
  .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
  .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
  .intr_alloc_flags = 0,
  .dma_buf_count = 4,
  .dma_buf_len = 1024,
  .use_apll = true //false
};

const i2s_pin_config_t pin_config = {
  .bck_io_num = I2S_SCK,
  .ws_io_num = I2S_WS,
  .data_out_num = I2S_SD
};

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

  pinMode(STEREO_CHANNEL_SELECT, INPUT_PULLUP); // LPT pin 1 (_strobe)
  pinMode(26, OUTPUT); digitalWrite(26, LOW);

  /*Serial.begin(115200);
  while(Serial.available());
  Serial.println(); Serial.print("--- (compilation date: "); Serial.print(__DATE__); Serial.print(" "); Serial.print(__TIME__); Serial.println(") ---");
  */

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_start(I2S_NUM_0);

  //xTaskCreatePinnedToCore(core0_task_covox_stereo, "core0_task_covox_stereo", 4096, NULL, 5, NULL, 0);
  //attachInterrupt(I2S_WS, isr_sample_covox_stereo, RISING);
  if (rtc_get_reset_reason(0) == 5) {
    //Serial.println("stereo");
    //delay(1000);
    xTaskCreatePinnedToCore(core0_task_covox_stereo, "core0_task_covox_stereo", 4096, NULL, 5, NULL, 0);
    attachInterrupt(I2S_WS, isr_sample_covox_stereo, RISING);
  } else {
    //Serial.println("mono");
    //delay(1000);
    attachInterrupt(STEREO_CHANNEL_SELECT, isr_channelselect, RISING);
  }

}

void loop() {

  if (buffer_full) {
    esp_err_t result = ESP_OK;
    size_t bytesWritten;
    if (buffer_full == 1) {
      result = i2s_write(I2S_NUM_0, &buf[0], SIZE_OF_COVOX_BUF_IN_BYTES/2, &bytesWritten, portMAX_DELAY);
    }
    if (buffer_full == 2) {
      result = i2s_write(I2S_NUM_0, &buf[1024], SIZE_OF_COVOX_BUF_IN_BYTES/2, &bytesWritten, portMAX_DELAY);
    }
    if (result != ESP_OK) Serial.println("error in i2s_write (covox)");
    totalSamplesPlayed += bytesWritten/4;
    buffer_full = 0;
  } // COVOX

  static uint32_t oldtime = 0, newtime = 0, last = 0, oldcnt = 1111;
  /*newtime = micros();
  //if ( (newtime-oldtime) > 100000 ) {
  //if ( (newtime-oldtime) > 1000000 ) {
  if ( newtime > oldtime ) {
    //Serial.print(totalSampleCounter*4-totalSamplesPlayed); Serial.print(" / "); Serial.println(totalFifoInterruptCounter);
    //Serial.print(mode); Serial.print(" "); Serial.print(modeA); Serial.print(" "); Serial.print(modeB); Serial.print(" "); Serial.println(totalSamplesPlayed);
    //Serial.println(totalChannelInterruptCounter-last); last = totalChannelInterruptCounter;
    //Serial.println(millis());
    //Serial.println(cycles);
    drawing = 1;
    for (uint32_t i = 0; i < 150; i++) {
      //Serial.print((s[i] >> STEREO_CHANNEL_SELECT)&1); Serial.print(" "); Serial.println(i/50.0);
      //Serial.print((sin(i/5.0)+1.0)/2.0); Serial.print(" "); Serial.println(i/50.0);
      Serial.print((s[i] >> STEREO_CHANNEL_SELECT)&1);
    }
    Serial.println();
    drawing = 0;
    oldtime += 100000;//oldtime = newtime;
  }*/
  /*if (cnt != oldcnt) {
    //for (uint32_t i = 0; i < 150; i++) Serial.print((s[i] >> STEREO_CHANNEL_SELECT)&1);
    Serial.print(" "); Serial.println(cnt);
    oldcnt = cnt;
  }*/

}
