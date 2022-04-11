#include "driver/i2s.h"

static const i2s_port_t i2s_num = I2S_NUM_0;

uint32_t buf[16];

volatile uint32_t totalInterruptCounter = 0;
uint32_t totalSamplesPlayed = 0;
#define FIFOFULL 22
#define FIFOCLK 21

static const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = 7000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,       // high interrupt priority = 1
    .dma_buf_count = 4,                             // 8 buffers
    .dma_buf_len = 16,                            // Number of frames for one-time sampling. The frame here means the total data from all the channels in a WS cycle
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
  pinMode(23, OUTPUT); digitalWrite(23, HIGH);
  pinMode(21, INPUT); // fifoclock, 17 (Select Printer_) (PC->DSS)
  pinMode(22, OUTPUT); digitalWrite(22, LOW); // fifofull, 10 (ACK) (DSS->PC)
  
  Serial.begin(115200);
  while(Serial.available());
  Serial.println("alku");

  i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
  //i2s_set_pin(i2s_num, &pin_config);
  i2s_set_pin(i2s_num, NULL);

  Serial.println("setup():n loppu");
}


void loop()
{
  size_t bytesWritten;
  i2s_write(i2s_num, &buf, sizeof(buf), &bytesWritten, portMAX_DELAY);
  digitalWrite(FIFOFULL, LOW);
  //while(!digitalRead(FIFOCLK));
  uint32_t t1 = micros();
  for (uint8_t i = 0; i < 16; i++) {
    while(digitalRead(FIFOCLK));
    //if (digitalRead(FIFOCLK)==0) Serial.println(micros());
    //Serial.println(digitalRead(FIFOCLK));
    //delayMicroseconds(1);
    uint8_t value = (REG_READ(GPIO_IN_REG) >> 12);
    buf[i] = (value<<24) | (value<<8);
    //Serial.println(digitalRead(FIFOCLK));
    //delayMicroseconds(3);
    //if (i != 15) while(!digitalRead(FIFOCLK));
    //while(!digitalRead(FIFOCLK));
    while(!digitalRead(FIFOCLK));
  }
  uint32_t t2 = micros();
  digitalWrite(FIFOFULL, HIGH);
  //delayMicroseconds(500);
  Serial.println(t2-t1);
  //Serial.println((uint8_t)(buf[0] >> 8));
  //Serial.print("t: "); Serial.print(t2-t1); Serial.print(" I-S: "); Serial.println(totalInterruptCounter-totalSamplesPlayed);

  /*uint8_t value = (REG_READ(GPIO_IN_REG) >> 12);
  Serial.println(value);*/
}
