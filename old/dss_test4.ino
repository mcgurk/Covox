#include "driver/i2s.h"

static const i2s_port_t i2s_num = I2S_NUM_0;

#define FIFOFULL 22
#define FIFOCLK 21

uint32_t buf[1024];
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t totalInterruptCounter = 0;
volatile uint32_t totalFifoInterruptCounter = 0;
volatile uint8_t buffer_full = 0;
volatile int8_t fifo_idx = 0;
//volatile uint8_t fifo_first = 0;
//volatile uint8_t fifo_last = 0;
volatile uint8_t fifo_buf[16];
uint32_t totalSamplesPlayed = 0;

void IRAM_ATTR isr_fifo() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (fifo_idx < 16) fifo_buf[fifo_idx++] = (REG_READ(GPIO_IN_REG) >> 12);
  if (fifo_idx > 15) digitalWrite(FIFOFULL, HIGH);
  totalFifoInterruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  int8_t s;
  if (fifo_idx > 0) 
    s = fifo_buf[--fifo_idx];
  else
    s = 128;
  uint16_t i = totalInterruptCounter & 1023;
  buf[i] = (s<<24) | (s<<8);
  if (i == 511) buffer_full = 1;
  if (i == 1023) buffer_full = 2;
  if (fifo_idx < 14) digitalWrite(FIFOFULL, LOW);
  //portENTER_CRITICAL_ISR(&timerMux);
  totalInterruptCounter++;
  //portEXIT_CRITICAL_ISR(&timerMux);
  portEXIT_CRITICAL_ISR(&timerMux);
}

static const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = 7000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,  // high interrupt priority
    .dma_buf_count = 8,                        // 8 buffers
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
  pinMode(21, INPUT); // fifoclock, 17 (Select Printer_) (PC->DSS)
  pinMode(22, OUTPUT); digitalWrite(22, LOW); // fifofull, 10 (ACK) (DSS->PC)
  pinMode(23, OUTPUT); digitalWrite(23, HIGH); // 5V
  pinMode(32, OUTPUT); digitalWrite(32, LOW); // GND

  Serial.begin(115200);
  while(Serial.available());
  Serial.println("alku");
  
  i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
  //i2s_set_pin(i2s_num, &pin_config);
  i2s_set_pin(i2s_num, NULL);
  
  timer = timerBegin(0, 3, true); // 1 = prescaler
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 3808, true); // 80 000 000 / 11 428 = 7000.350018...kHz, 80 000 000 / 11 429 = 6999.7375...kHz
  //prescaler 1 & count 1000 -> 39903...39904, (2:lla sama?!), pre3&c1000->26633, 11429 -> 3499,
  //pre3&c3805->7005..7006, pre3&c3810->6996..6997, pre3&3809->6998..6999
  //pre3&c3809 puskuri tyhjenee ehkä 1 sample per sekuntti.
  //pre3&c3808 puskuri pysyy synkissä 7kHz:n DAC:n kanssa!!!
  Serial.println("ennen timerin enablointia");
  timerAlarmEnable(timer);

  attachInterrupt(21, isr_fifo, RISING);
  
  Serial.println("setup():n loppu");
}


uint32_t oldtime = 0, newtime = 0;
uint32_t oldintcount = 0, newintcount = 0;

void loop() {

  if (buffer_full) {
    size_t bytesWritten;
    //uint32_t i = totalInterruptCounter;
    //uint16_t c = i & 1023;
    if (buffer_full == 1) i2s_write(i2s_num, &buf[0], sizeof(buf)/2, &bytesWritten, 100);
    if (buffer_full == 2) i2s_write(i2s_num, &buf[512], sizeof(buf)/2, &bytesWritten, 100);
    totalSamplesPlayed += bytesWritten/4;
    buffer_full = 0;
  }

  newtime = micros();
  newintcount = totalInterruptCounter;
  //if ( (newtime-oldtime) > 1000000 ) {
  if ( (newtime-oldtime) > 10000 ) {
    uint32_t ints = newintcount - oldintcount;
    //Serial.println(ints);
    Serial.print(totalFifoInterruptCounter); Serial.print(" "); Serial.println(fifo_idx);
    oldtime = newtime;
    oldintcount = newintcount;
  }
  //Serial.print("B: "); Serial.print(bytesWritten); Serial.print(" I-S: "); Serial.println(totalInterruptCounter-totalSamplesPlayed+100);
  //Serial.println(totalInterruptCounter-totalSamplesPlayed+100);
}
