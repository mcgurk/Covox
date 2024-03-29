#include <driver/i2s.h>

#define I2S_WS 33
#define I2S_SD 23
#define I2S_SCK 32
//#define SAMPLE_RATE 7000
#define SAMPLE_RATE 48000

#define bufferLen 1024
int16_t sBuffer[bufferLen];

const i2s_config_t i2s_config = {
  .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX ),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = i2s_bits_per_sample_t(16),
  .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
  .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
  .intr_alloc_flags = 0,
  .dma_buf_count = 8,
  .dma_buf_len = bufferLen,
  .use_apll = false
};

const i2s_pin_config_t pin_config = {
  .bck_io_num = I2S_SCK,
  .ws_io_num = I2S_WS,
  .data_out_num = -1, //I2S_SD_tx,
  .data_in_num = I2S_SD
};

static const i2s_config_t i2s_config_dac = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_LSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,  // high interrupt priority
    .dma_buf_count = 8,                        // 8 buffers
    .dma_buf_len = bufferLen,                  // Number of frames for one-time sampling. The frame here means the total data from all the channels in a WS cycle
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = -1
};

volatile uint32_t wscount;

void IRAM_ATTR isr_ws() {
  wscount++;
}

//static QueueHandle_t i2s_event_queue_dac;

void setup() {
  Serial.begin(115200);
  while(!Serial);

  i2s_driver_install(I2S_NUM_1, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &pin_config);
  i2s_start(I2S_NUM_1);

  i2s_driver_install(I2S_NUM_0, &i2s_config_dac, 0, NULL);
  //i2s_driver_install(I2S_NUM_0, &i2s_config_dac, 10, &i2s_event_queue_dac);
  i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
  
  attachInterrupt(I2S_WS, isr_ws, RISING);

}

void loop() {

  size_t bytesIn, bytesOut;
  static uint32_t bytesread = 0, byteswritten = 0;
  esp_err_t result;
  
  /*result = i2s_read(I2S_NUM_1, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);
  //result = i2s_read(I2S_NUM_1, &sBuffer, bufferLen, &bytesIn, 1);
  if (result != ESP_OK) Serial.println("error in i2s_read");
  bytesread += bytesIn;*/

  /*result = i2s_write(I2S_NUM_0, &sBuffer, bufferLen, &bytesOut, portMAX_DELAY);
  i2s_event_t i2s_event;
  if (xQueueReceive(i2s_event_queue_dac, &i2s_event, portMAX_DELAY) != pdFALSE) {
    if (i2s_event.type == I2S_EVENT_TX_Q_OVF) Serial.println("*TX_Q_OVF*");
    if (i2s_event.type == I2S_EVENT_TX_DONE) Serial.println("*TX_DONE*");
  }*/

  //result = i2s_write(I2S_NUM_0, &sBuffer, bytesIn, &bytesOut, portMAX_DELAY);
  //result = i2s_write(I2S_NUM_0, &sBuffer, bytesIn, &bytesOut, 1);
  result = i2s_write(I2S_NUM_0, &sBuffer, bufferLen, &bytesOut, portMAX_DELAY);
  if (result != ESP_OK) Serial.println("error in i2s_write");
  byteswritten += bytesOut;

  static uint32_t old_time, new_time, old_samplecnt_dac, old_samplecnt_i2s, old_samplecnt_i2s_isr;
  new_time = millis();
  if ( (new_time-old_time) > 1000 ) {
    //Serial.println(bytesread-byteswritten);
    //Serial.println(bytesread-(wscount*4));
    //Serial.println((wscount*4)-byteswritten);
    /*Serial.print("dac: "); Serial.println((byteswritten - old_samplecnt_dac)/4);
    old_samplecnt_dac = byteswritten;
    Serial.print("i2s: "); Serial.println((bytesread - old_samplecnt_i2s)/4);
    old_samplecnt_i2s = bytesread;
    Serial.print("i2s_isr: "); Serial.println(((wscount*4) - old_samplecnt_i2s_isr)/4);
    old_samplecnt_i2s_isr = (wscount*4);*/

    //Serial.print("i2s: "); Serial.println(bytesread);
    //Serial.print("i2s_isr: "); Serial.println(wscount*4);
    //Serial.println(bytesread-(wscount*4)); // vakio
    Serial.println(byteswritten-(wscount*4)); // ei vakio :( (3-4s välein 1samplen heitto; dac on nopeampi kuin i2s)(apll kääntää tilanteen)

    //Serial.println(wscount);
    old_time = new_time;
  }
}
