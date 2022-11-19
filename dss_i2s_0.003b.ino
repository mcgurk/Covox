// ESP32 has DAC on GPIO pins 25 and 26

#include <I2S.h>

#define FIFOFULL 22
#define FIFOCLK 21

hw_timer_t * timer = NULL;

volatile uint8_t fifo_buf[32];
volatile uint32_t totalTimerInterruptCounter = 0;
volatile uint32_t totalFifoInterruptCounter = 0;
volatile uint8_t front = 0;
volatile uint8_t back = 0;
#define fsize ((uint8_t)(back-front)) // 0-16

void IRAM_ATTR isr_fifo() {
  if (fsize < 16) fifo_buf[(back++)&31] = (REG_READ(GPIO_IN_REG) >> 12); // if there is free space in buffer, read sample
  if (fsize == 16) digitalWrite(FIFOFULL, HIGH); // buffer is full, rise the full flag
  totalFifoInterruptCounter++;
}

void IRAM_ATTR onTimer() {
  uint8_t s;
  if (fsize > 0) s = fifo_buf[(front++)&31]; else s = 128; // if fifo is empty, play silence (128/0x80)
  if (fsize < 15) digitalWrite(FIFOFULL, LOW); // got one sample away from buffer so there is free space again, lower the full flag
  uint16_t sample = s<<8;
  I2S.write(sample); // Right channel
  I2S.write(0x80<<8); // Left channel // "silence"
  totalTimerInterruptCounter++;
}

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
  Serial.println(); Serial.print("--- (compilation date: "); Serial.print(__DATE__); Serial.print(" "); Serial.print(__TIME__); Serial.println(") ---");
  
  I2S.begin(ADC_DAC_MODE, 7000, 16);
    
  timer = timerBegin(0, 3, true); // 3 = prescaler
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 3808, true);
  // with prescaler 3 and count 3808 internal DAC buffer is syncronized with timer interrupt at 7kHz.
  timerAlarmEnable(timer);

  // "The rising edge of the pulse on Pin 17 from the printer interface is used to clock data into the FIFO"
  attachInterrupt(21, isr_fifo, RISING);
}


uint32_t oldtime = 0, newtime = 0;

void loop() {

  newtime = micros();
  if ( (newtime-oldtime) > 10000 ) {
    Serial.println(I2S.availableForWrite());
    oldtime = newtime;
  }

}
