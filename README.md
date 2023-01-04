# NOTICE!
At the moment (7.12.2022, newest ESP32 Adruino IDE board 2.0.5) I cannot get internal DAC to work. Workaround: I use ESP32 Arduino IDE board 1.0.6.


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
IO16 ( $\colorbox{brown}{{\color{brown}{brown}}}$ ) | 2 (data0)
IO17 ( $\colorbox{red}{{\color{red}{red}}}$ ) | 3 (data1)
IO18 ( $\colorbox{orange}{{\color{orange}{orange}}}$ ) | 4 (data2)
IO19 ( $\colorbox{yellow}{{\color{yellow}{yellow}}}$ ) | 5 (data3)
IO4  ( $\colorbox{green}{{\color{green}{green}}}$ ) | 6 (data4)
IO21 ( $\colorbox{blue}{{\color{blue}{blue}}}$ ) | 7 (data5)
IO22 ( $\colorbox{purple}{{\color{purple}{purple}}}$ ) | 8 (data6)
IO23 ( $\colorbox{gray}{{\color{gray}{gray}}}$ ) | 9 (data7)
GND | GND (e.g. 25)
**DSS:** | 
IO21 | 17 (Select Printer_) (PC->DSS)
IO22 | 10 (ACK) (DSS->PC)
**StereoIn1Covox:** | 
? | 1 (Strobe_) (channel select PC->Covox)
**Amplifier:** |
5V | 2-5VDD
GND | Ground
IO25 | Audio In-
IO26 | Audio In+

**Remember to do 5V->3.3V levelshifting! ESP32 is not 5V tolerant!**

# Misc stuff

###
```
// .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT
buf[i] = (0x80<<24) | (s<<8); // IO26 = 0x80, IO25 = sample
```

#### ASM
```
C:\Users\lehti\AppData\Local\Arduino15\packages\esp32\tools\xtensa-esp32-elf-gcc\1.22.0-97-gc752ad5-5.2.0\bin\xtensa-esp32-elf-objdump.exe -S C:\Users\lehti\AppData\Local\Temp\arduino_build_606462\esp32_dac_test2.ino.elf > c:\temp\koe.txt
```

## Links
- https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
- https://www.weigu.lu/microcontroller/tips_tricks/esp32_tips_tricks/index.html
- https://www.mischianti.org/wp-content/uploads/2020/11/ESP32-DOIT-DEV-KIT-v1-pinout-mischianti.png
- https://www.mischianti.org/wp-content/uploads/2021/07/ESP32-DEV-KIT-DevKitC-v4-pinout-mischianti.png
- DSS support: https://www.mobygames.com/attribute/sheet/attributeId,44/p,2/
- Covox support: https://www.mobygames.com/attribute/sheet/attributeId,43/p,2/
- Adafruit Mono 2.5W Class D Audio Amplifier - PAM8302: https://www.adafruit.com/product/2130
- Miles Sound System (MSS) DSS patch: https://www.vogons.org/viewtopic.php?t=69539
- Disney Sound Source driver for Win3x, Win9x, WinNT 4: http://www.vogonsdrivers.com/getfile.php?fileid=1680&menustate=0
- DSS Programmers Guide: https://archive.org/details/dss-programmers-guide/page/n1/mode/2up
- Reversing the Disney Sound Source: https://www.vogons.org/viewtopic.php?f=62&t=42250&start=140
- DIY DSS: https://www.vogons.org/viewtopic.php?p=474360#p474360
- ESP32 DAC: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/dac.html

## Problems
- Commander Keen Dreams doesn't detect this DSS. Why?
- Reorganize pins. ESP32 doesn't boot cleanly when connected. Is IO14 and IO15 problem?

### DSS checks
- Dungeon Master DSS detection: https://www.vogons.org/viewtopic.php?t=40751
- Wolfenstein 3D writes 32 samples to FIFO and checks that FIFOFULL pin activates

## To do
- Stereo in 1: Crystal Dream https://www.pouet.net/prod.php?which=463#c173793 (LPT pin 1)

## KDreams
- doesn't detect
- 33 sample interrupts comes

## DosBox
```
[speaker]
lpt_dac = disney, covox, ston1, (or none/off)
```
## Old garbage

ESP32 DAC output: 25, GND

(to filter DC out put e.g. 10uF capacitor between 25 and out+. to filter noise put 560ohm resistor between 25 and 10uF capacitor and 0.01...0.1uF capacitor between 560ohm and gnd)

(if signal is too high, make resistor divider with two resistor (e.g. 2kohm))

Old pinout was IO12..IO19
