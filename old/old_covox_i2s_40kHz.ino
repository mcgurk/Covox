#include "driver/i2s.h"

static const i2s_port_t i2s_num = I2S_NUM_0;

TaskHandle_t Task1;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

uint32_t buf[1024];
volatile uint32_t totalInterruptCounter = 0;
volatile uint8_t buffer_full = 0;
uint32_t totalSamplesPlayed = 0;

void IRAM_ATTR onTimer() {
  uint8_t s1 = (REG_READ(GPIO_IN_REG) >> 12);
  uint8_t s2 = (REG_READ(GPIO_IN_REG) >> 12);
  uint8_t s3 = (REG_READ(GPIO_IN_REG) >> 12);
  uint8_t value;
  if (s1 == s2) 
    value = s2;
  else
    value = s3;
  uint16_t i = totalInterruptCounter & 1023;
  buf[i] = (value<<24) | (value<<8); 
  if (i == 511) buffer_full = 1;
  if (i == 1023) buffer_full = 2;
  //portENTER_CRITICAL_ISR(&timerMux);
  totalInterruptCounter++;
  //portEXIT_CRITICAL_ISR(&timerMux);
}

static const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = 40000,
    //.sample_rate = 7000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    //.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,  // high interrupt priority
    .dma_buf_count = 4,                        // 8 buffers
    .dma_buf_len = 512,                        // Number of frames for one-time sampling. The frame here means the total data from all the channels in a WS cycle
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
  pinMode(32, OUTPUT); digitalWrite(32, LOW); // GND

  Serial.begin(115200);
  while(Serial.available());
  Serial.println("start");
  
  i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
  //i2s_set_pin(i2s_num, &pin_config);
  i2s_set_pin(i2s_num, NULL);
  
  //timer = timerBegin(0, 2, true);  // 80 (using 80 as the prescaler value), we will get a signal with a 1 MHz frequency that will increment the timer counter 1 000 000 times per second.
  timer = timerBegin(0, 2, true); // 0,1,2 kertoimet sama asia? 0 ei toimi?
  timerAttachInterrupt(timer, &onTimer, true);
  //timerAlarmWrite(timer, 604, true); // 44kHz 3: 800/700 laskee, 600 nousee 4, 610 laskee 4, 605 laskee alle 1, 604 nousee alle 1
  //timerAlarmWrite(timer, 907, true); // 44kHz 2/1: 905/906 nousee alle 1, 907 vähenee alle 1
  timerAlarmWrite(timer, 997, true); // 40kHz 2: 997 spot on!
  //timerAlarmWrite(timer, 3808, true);
  //timerAlarmWrite(timer, 3808, true);
  
  Serial.println("before enabling timer");
  timerAlarmEnable(timer);

  Serial.println("end of setup()");
}

uint32_t oldtime = 0, newtime = 0;
uint32_t oldintcount = 0, newintcount = 0;

void loop() {
  if (buffer_full) {
    size_t bytesWritten;
    if (buffer_full == 1) i2s_write(i2s_num, &buf[0], sizeof(buf)/2, &bytesWritten, 100);
    if (buffer_full == 2) i2s_write(i2s_num, &buf[512], sizeof(buf)/2, &bytesWritten, 100);
    //i2s_write(i2s_num, &buf[0], sizeof(buf)/2, &bytesWritten, portMAX_DELAY);
    totalSamplesPlayed += bytesWritten/4;
    buffer_full = 0;
  }

  newtime = micros();
  newintcount = totalInterruptCounter;
  if ( (newtime-oldtime) > 10000 ) {
    uint32_t ints = newintcount - oldintcount;
    //Serial.println(samples_count);
    //Serial.println(totalInterruptCounter-totalSamplesPlayed);
    oldtime = newtime;
    oldintcount = newintcount;
  }

}
