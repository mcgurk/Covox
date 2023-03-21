22.3.2023 0.40
 ```
#define VOLUME 4 // 0 min, 8 max
#define DEBUG
//#define EXTRA_GND 26

//COVOX
#define D0 26 // white
#define D1 13 // grey
#define D2 14 // yellow
#define D3 27 // brown
#define D4  9 // blue
#define D5 10 // purple
#define D6 18 // pink
#define D7 23 // green

#define STEREO_CHANNEL_SELECT 25
#define STEREO_CHANNEL_SELECT_PULLUP 4

//DSS
#define FIFOCLK 19 // fifoclock, 17 (Select Printer_) (PC->DSS)
#define FIFOFULL 22 // fifofull, 10 (ACK) (DSS->PC)

// I2S:
#define I2S_WS 21 // 19
#define I2S_SCK 33 // 22
#define I2S_SD 32 //21
#define SAMPLE_RATE_DSS 14000
#define SAMPLE_RATE_COVOX 96000
```
