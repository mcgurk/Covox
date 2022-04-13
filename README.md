# Covox
Trying to make lpt soundcard Covox with microcontroller

## Wiring

ESP32 DAC output: 25, GND

(to filter DC out put e.g. 10uF capacitor between 25 and out+. to filter noise put 0.1uF capacitor between 25 and gnd) 

ESP32 | LPT (D25)
--- | ---
**Covox:** | 
12 | 2 (data0)
13 | 3 (data1)
14 | 4 (data2)
15 | 5 (data3)
16 | 6 (data4)
17 | 7 (data5)
18 | 8 (data6)
19 | 9 (data7)
GND | GND (e.g. 25)
**DSS:** | 
21 | 17 (Select Printer_) (PC->DSS)
22 | 10 (ACK) (DSS->PC)

**Remember to do 5V->3.3V levelshifting! ESP32 is not 5V tolerant!**

# Misc stuff

#### ASM
```
C:\Users\lehti\AppData\Local\Arduino15\packages\esp32\tools\xtensa-esp32-elf-gcc\1.22.0-97-gc752ad5-5.2.0\bin\xtensa-esp32-elf-objdump.exe -S C:\Users\lehti\AppData\Local\Temp\arduino_build_606462\esp32_dac_test2.ino.elf > c:\temp\koe.txt
```
