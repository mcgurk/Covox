#include "driver/i2s.h"

static const i2s_port_t i2s_num = I2S_NUM_0;

//TaskHandle_t m_i2sWriterTaskHandle;
//QueueHandle_t m_i2sQueue;

uint32_t buf[4096];
//uint32_t buf[1024];

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t totalInterruptCounter = 0;
uint32_t totalSamplesPlayed = 0;

void IRAM_ATTR onTimer() {
  
  int32_t gpio = REG_READ(GPIO_IN_REG);
  uint8_t value = (gpio >> 12);
  //uint16_t i = totalInterruptCounter & 4095; // 0-2047
  uint16_t i = totalInterruptCounter & 4095; // 0-2047
  uint32_t o = (value<<24) | (value<<8); 

  portENTER_CRITICAL_ISR(&timerMux);
  buf[i] = o;
  totalInterruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

static const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    //.sample_rate = 44100,
    .sample_rate = 100000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    //.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,       // high interrupt priority = 1
    .dma_buf_count = 8,                             // 8 buffers
    .dma_buf_len = 1024,                            // Number of frames for one-time sampling. The frame here means the total data from all the channels in a WS cycle
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

  Serial.begin(115200);
  while(Serial.available());
  Serial.println("alku");
  

  //xTaskCreate(i2sWriterTask, "i2s Writer Task", 4096, NULL, 1, &m_i2sWriterTaskHandle);
  
  //timer = timerBegin(0, 80, true);  // 80 (using 80 as the prescaler value), we will get a signal with a 1 MHz frequency that will increment the timer counter 1 000 000 times per second.
  timer = timerBegin(0, 80, true);  // 80 (using 80 as the prescaler value), we will get a signal with a 1 MHz frequency that will increment the timer counter 1 000 000 times per second.
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10, true); // 25 -> 40kHz, 1MHz / 50 = 20kHz // 1MHz / 20 = 50kHz //1MHz / 100 = 10kHz? // 16 -> 62.5kHz
  //timerAlarmWrite(timer, 23, true); // 1MHz / 44100 = ~23, 1MHz / 50 = 20kHz // 1MHz / 20 = 50kHz //1MHz / 100 = 10kHz?
  Serial.println("ennen timerin enablointia");
  timerAlarmEnable(timer);
  delay(500);

  //static QueueHandle_t i2s_event_queue;
  i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
  //i2s_driver_install(i2s_num, &i2s_config, 10, &i2s_event_queue);
  //i2s_set_pin(i2s_num, &pin_config);
  i2s_set_pin(i2s_num, NULL);

  Serial.println("setup():n loppu");
}



void loop()
{

  size_t bytesWritten;
  portENTER_CRITICAL(&timerMux);
  uint32_t i = totalInterruptCounter;
  portEXIT_CRITICAL(&timerMux);
  uint16_t c = i & 4095; // 0-2047
  uint16_t b = sizeof(buf)/4;
  if (c < 1024)
    i2s_write(i2s_num, &buf[2048], b, &bytesWritten, portMAX_DELAY);
  else if (c < 2048)
    i2s_write(i2s_num, &buf[3072], b, &bytesWritten, portMAX_DELAY);
  else if (c < 3071)
    i2s_write(i2s_num, &buf[0], b, &bytesWritten, portMAX_DELAY);
  else
    i2s_write(i2s_num, &buf[1024], b, &bytesWritten, portMAX_DELAY);
  //totalSamplesPlayed += sizeof(buf)/2;
  if (bytesWritten != b) {
    Serial.println("VIRHE!");
    delay(1000);
  }
  totalSamplesPlayed += bytesWritten/4;
  //totalSamplesPlayed += 256;
  //Serial.print("B: "); Serial.print(bytesWritten); Serial.print(" I: "); Serial.print(totalInterruptCounter); Serial.print(" S: ");Serial.println(totalSamplesPlayed);
  //Serial.print("B: "); Serial.print(bytesWritten); Serial.print(" I: "); Serial.print(totalInterruptCounter); Serial.print(" S: ");Serial.println(totalSamplesPlayed);
  //Serial.print("B: "); Serial.print(bytesWritten); Serial.print(" I-S: "); Serial.println(totalInterruptCounter-totalSamplesPlayed+100);
  Serial.print("b: "); Serial.print(b); Serial.print(" I-S: "); Serial.println(totalInterruptCounter-totalSamplesPlayed);
  //Serial.println(totalInterruptCounter-totalSamplesPlayed);

}
