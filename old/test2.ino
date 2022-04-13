#include "driver/i2s.h"

static const i2s_port_t i2s_num = I2S_NUM_0;

//TaskHandle_t m_i2sWriterTaskHandle;
//QueueHandle_t m_i2sQueue;

uint32_t buf[2048];
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t totalInterruptCounter = 0;
uint32_t totalSamplesPlayed = 0;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  totalInterruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
  
  int32_t gpio = REG_READ(GPIO_IN_REG);
  uint8_t value = (gpio >> 12);
  uint16_t i = totalInterruptCounter & 2047; // 0-2047
  buf[i] = (value<<24) | (value<<8);
}

volatile uint8_t semaphore = 0;

/*static const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,       // high interrupt priority
    .dma_buf_count = 8,                             // 8 buffers
    .dma_buf_len = 1024,                            // 1K per buffer, so 8K of buffer space
    .use_apll=0,
    .tx_desc_auto_clear= true, 
    .fixed_mclk=-1    
};

static const i2s_pin_config_t pin_config = {
    .bck_io_num = 27,                                 // The bit clock connectiom, goes to pin 27 of ESP32
    .ws_io_num = 26,                                  // Word select, also known as word select or left right clock
    .data_out_num = 25,                               // Data out from the ESP32, connect to DIN on 38357A
    .data_in_num = I2S_PIN_NO_CHANGE                  // we are not interested in I2S data into the ESP32
};*/

/*static const i2s_config_t i2s_config = {
    .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN,
    .sample_rate = 44100,
    .bits_per_sample = 16, // the DAC module will only take the 8bits from MSB
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .intr_alloc_flags = 0, // default interrupt priority
    .dma_desc_num = 8,
    .dma_frame_num = 64,
    .use_apll = false
};*/

/*static const i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN,
        //.mode = I2S_MODE_MASTER | I2S_MODE_TX,              // only TX
        .sample_rate = 44100,
        .bits_per_sample = 16,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,       // 2-channels
        //.communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .dma_buf_count = 6,
        .dma_buf_len = 60,
        .intr_alloc_flags = 0,                              // default interrupt priority
        .tx_desc_auto_clear = true                          // auto clear tx descriptor on underflow
};*/

static const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    //.sample_rate = 44100,
    .sample_rate = 100000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    //.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,       // high interrupt priority
    .dma_buf_count = 8,                             // 8 buffers
    .dma_buf_len = 512,                            // Number of frames for one-time sampling. The frame here means the total data from all the channels in a WS cycle
    .use_apll = 0,
    .tx_desc_auto_clear = true,
    .fixed_mclk = -1    
};

//------------------------------------------------------------------------------------------------------------------------

/*void i2sWriterTask(void *param)
{
  size_t bytesWritten;
  while (true)
  {
    // wait for some data to be requested
    i2s_event_t evt;
    if (xQueueReceive(m_i2sQueue, &evt, portMAX_DELAY) == pdPASS)
    {
      if (evt.type == I2S_EVENT_TX_DONE) // I2S DMA finish sent 1 buffer
      {
        //i2s_write(I2S_NUM_0, sawTooth + buffer_position, availableSamples * 2, &bytesWritten, portMAX_DELAY);
        i2s_write(I2S_NUM_0, buf, sizeof(buf), &bytesWritten, portMAX_DELAY);
        Serial.println(micros());
        semaphore = 1;
      }
    }
  }
}*/
    
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
  
  static QueueHandle_t i2s_event_queue;
  i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
  //i2s_driver_install(i2s_num, &i2s_config, 10, &i2s_event_queue);
  //i2s_set_pin(i2s_num, &pin_config);
  i2s_set_pin(i2s_num, NULL);

  //xTaskCreate(i2sWriterTask, "i2s Writer Task", 4096, NULL, 1, &m_i2sWriterTaskHandle);
  
  timer = timerBegin(0, 80, true);  // 80 (using 80 as the prescaler value), we will get a signal with a 1 MHz frequency that will increment the timer counter 1 000 000 times per second.
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10, true); // 25 -> 40kHz, 1MHz / 50 = 20kHz // 1MHz / 20 = 50kHz //1MHz / 100 = 10kHz? // 16 -> 62.5kHz
  //timerAlarmWrite(timer, 23, true); // 1MHz / 44100 = ~23, 1MHz / 50 = 20kHz // 1MHz / 20 = 50kHz //1MHz / 100 = 10kHz?
  Serial.println("ennen timerin enablointia");
  timerAlarmEnable(timer);
  
  Serial.println("setup():n loppu");
}



void loop()
{
  /*for (int i=0; i < 1*1024; i++) {
    uint8_t c = (REG_READ(GPIO_IN_REG) >> 12);
    buf[i] = (c<<24) | (c<<8);
    delayMicroseconds(20);
  }*/
  //while (semaphore == 0) yield();
  //semaphore = 0;
  //Serial.println(micros());
  size_t bytesWritten;
  //i2s_write(i2s_num, &buf, sizeof(buf), &BytesWritten, portMAX_DELAY);
  //i2s_write(i2s_num, &buf, sizeof(buf), &BytesWritten, 100);
  //portENTER_CRITICAL(&timerMux);
  uint32_t i = totalInterruptCounter;
  //portEXIT_CRITICAL(&timerMux);
  uint16_t c = i & 2047; // 0-2047
  if (c < 511)
    i2s_write(i2s_num, &buf[1024], sizeof(buf)/4, &bytesWritten, portMAX_DELAY);
  else if (c < 1023)
    i2s_write(i2s_num, &buf[1536], sizeof(buf)/4, &bytesWritten, portMAX_DELAY);
  else if (c < 1535)
    i2s_write(i2s_num, &buf[0], sizeof(buf)/4, &bytesWritten, portMAX_DELAY);
  else
    i2s_write(i2s_num, &buf[512], sizeof(buf)/4, &bytesWritten, portMAX_DELAY);
  //totalSamplesPlayed += sizeof(buf)/2;
  totalSamplesPlayed += 512;
  //Serial.print("B: "); Serial.print(bytesWritten); Serial.print(" I: "); Serial.print(totalInterruptCounter); Serial.print(" S: ");Serial.println(totalSamplesPlayed);
  //Serial.print("B: "); Serial.print(bytesWritten); Serial.print(" I: "); Serial.print(totalInterruptCounter); Serial.print(" S: ");Serial.println(totalSamplesPlayed);
  //Serial.print("B: "); Serial.print(bytesWritten); Serial.print(" I-S: "); Serial.println(totalInterruptCounter-totalSamplesPlayed+100);
  //Serial.print("c: "); Serial.print(c); Serial.print(" I-S: "); Serial.println(totalInterruptCounter-totalSamplesPlayed+100);
/*  //static uint8_t value = 0;
  uint8_t value = (REG_READ(GPIO_IN_REG) >> 12);
  uint32_t Value32Bit = (((uint32_t)value)<<24) | (((uint32_t)value)<<8);
  //i2s_write(i2s_num, &Value32Bit, 4, &BytesWritten, 100);
  i2s_write(i2s_num, &Value32Bit, 4, &BytesWritten, portMAX_DELAY);
  //i2s_write(i2s_num, &Value32Bit, 4, &BytesWritten, 0);
  //delayMicroseconds(20); //44100kHz -> 0,000022675736*/

}
