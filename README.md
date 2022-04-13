# Covox
Trying to make lpt soundcard Covox with microcontroller

## Wiring
ESP32 | LPT (D25)
--- | ---
12 | 2 (data0)
13 | 3 (data1)
14 | 4 (data2)
15 | 5 (data3)
16 | 6 (data4)
17 | 7 (data5)
18 | 8 (data6)
19 | 9 (data7)
GND | GND (e.g. 25)
Remember to do 5V->3.3V levelshifting! ESP32 is not 5V tolerant!

ESP32 DAC output: 25, GND

#### ASM
```
C:\Users\lehti\AppData\Local\Arduino15\packages\esp32\tools\xtensa-esp32-elf-gcc\1.22.0-97-gc752ad5-5.2.0\bin\xtensa-esp32-elf-objdump.exe -S C:\Users\lehti\AppData\Local\Temp\arduino_build_606462\esp32_dac_test2.ino.elf > c:\temp\koe.txt
```
