// As we're using an 8 bit value for the DAC 0-255 (256 parts) that means for us
// there are 256 'bits' to a complete circle not 360 (as in degrees) or even 2PI Radians
// (There are 2*PI Radians in a circle and computers love to work in Radians!)
// The computer works in radians for SIN, COSINE etc. so we must convert our 0 -255 value
// to radians, the comments in the code show this.

#include "soc/rtc_wdt.h"

volatile uint8_t interruptCounter = 0;
volatile uint32_t totalInterruptCounter = 0;

volatile int SineValues[256];       // an array to store our values for sine

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
 
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  totalInterruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
  
  int32_t gpio = REG_READ(GPIO_IN_REG);
  uint8_t value = (gpio >> 12);
  dacWrite(25, value);

  //dacWrite(25, SineValues[interruptCounter]);

}


void setup()
{
  Serial.begin(115200);
  while(Serial.available());

  Serial.println("alku");

  /*pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  pinMode(16, INPUT_PULLUP);
  pinMode(17, INPUT_PULLUP);
  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);*/
  pinMode(12, INPUT);
  pinMode(13, INPUT);
  pinMode(14, INPUT);
  pinMode(15, INPUT);
  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  float ConversionFactor=(2*PI)/256;        // convert my 0-255 bits in a circle to radians
                                            // there are 2 x PI radians in a circle hence the 2*PI
                                            // Then divide by 256 to get the value in radians
                                            // for one of my 0-255 bits.
  float RadAngle;                           // Angle in Radians
  for(int MyAngle=0;MyAngle<256;MyAngle++) {
    RadAngle=MyAngle*ConversionFactor*4;               // 8 bit angle converted to radians
    SineValues[MyAngle]=(sin(RadAngle)*127)+128;  // get the sine of this angle and 'shift' up so
  }

 
  /*timer = timerBegin(0, 80, true);  // 80 (using 80 as the prescaler value), we will get a signal with a 1 MHz frequency that will increment the timer counter 1 000 000 times per second.
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 50, true); // 1MHz / 50 = 20kHz // 1MHz / 20 = 50kHz //1MHz / 100 = 10kHz?
  Serial.println("ennen timerin enablointia");
  timerAlarmEnable(timer);*/

  rtc_wdt_protect_off();
  rtc_wdt_disable();

  Serial.println("setupin loppu");
}

uint8_t i = 0;

void loop()
{
  noInterrupts();
  while (1) {
    int32_t gpio = REG_READ(GPIO_IN_REG);
    uint8_t value = (gpio >> 12);
    dacWrite(25, value);
  }
  //for(int i=0;i<256;i++)
//    dacWrite(25,SineValues[i++]);

  /*int t1, t2;
  t1 = micros();
  for (int c=0; c<1000; c++) dacWrite(25, 0);
  t2 = micros();
  Serial.println(t2-t1);*/
  /*uint32_t value = totalInterruptCounter;
  if ( value > 20000 ) {
    Serial.print("Total number: ");
    Serial.println(value);
    portENTER_CRITICAL(&timerMux);
    totalInterruptCounter = 0;
    portEXIT_CRITICAL(&timerMux);
  }*/
  //Serial.println(totalInterruptCounter);
//  delay(1000);
  /*if (interruptCounter > 0) {
 
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
 
    totalInterruptCounter++;
 
    Serial.print("An interrupt as occurred. Total number: ");
    Serial.println(totalInterruptCounter);
 
  }*/

  /*uint32_t gpio = REG_READ(GPIO_IN_REG);
  uint8_t value = (gpio >> 12);
  Serial.println(value);
  delay(50);*/
  /*uint32_t gpio = REG_READ(GPIO_IN_REG);
  Serial.println(gpio, BIN);
  delay(50);*/

}
