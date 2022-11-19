# Covox
Trying to make lpt soundcard Covox with microcontroller

## Parts

- ESP32 dev board or Wemos D1 mini ESP32
- 12 channel level shifter (min. 8 channel for covox)
- Mono 2.5W Class D Audio Amplifier - PAM8302
- Speaker (small shelf speaker is great choice)

## Wiring

ESP32 | LPT (D25)
--- | ---
Differential audio out 25 & 26 | 
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
**Amplifier:** |
5V | 2-5VDD
GND | Ground
IO25 | 	Audio In-
IO26 | Audio In+

**Remember to do 5V->3.3V levelshifting! ESP32 is not 5V tolerant!**

# Misc stuff

###
```
// .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT
buf[i] = (0x80<<24) | (s<<8); // IO25 = 0x80, IO26 = sample
```

#### ASM
```
C:\Users\lehti\AppData\Local\Arduino15\packages\esp32\tools\xtensa-esp32-elf-gcc\1.22.0-97-gc752ad5-5.2.0\bin\xtensa-esp32-elf-objdump.exe -S C:\Users\lehti\AppData\Local\Temp\arduino_build_606462\esp32_dac_test2.ino.elf > c:\temp\koe.txt
```

## Links
- https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
- DSS support: https://www.mobygames.com/attribute/sheet/attributeId,44/p,2/
- Covox support: https://www.mobygames.com/attribute/sheet/attributeId,43/p,2/
- Adafruit Mono 2.5W Class D Audio Amplifier - PAM8302: https://www.adafruit.com/product/2130

## Problems
- Commander Keen Dreams doesn't detect this DSS. Why?
- Reorganize pins. ESP32 doesn't boot cleanly when connected. Is IO14 and IO15 problem?

## To do
- Stereo in 1: Crystal Dream https://www.pouet.net/prod.php?which=463#c173793 (LPT pin 1)

## Old garbage

ESP32 DAC output: 25, GND

(to filter DC out put e.g. 10uF capacitor between 25 and out+. to filter noise put 560ohm resistor between 25 and 10uF capacitor and 0.01...0.1uF capacitor between 560ohm and gnd)

(if signal is too high, make resistor divider with two resistor (e.g. 2kohm))
