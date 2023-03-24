
# Covox, Disney Sound Source (DSS), Stereo in 1
Trying to make parallel port / LPT soundcard Covox with microcontroller

## Parts

- ESP32
- I2S DAC (e.g. PCM5102A/GY-PCM5102 or I2S DAC amplifier MAX98357A)

## Wiring

ESP32 | LPT (D25)
--- | ---
**Covox:** |
IO13 ( $\colorbox{white}{{\color{white}{white}}}$ ) | 2 (D0)
IO14 ( $\colorbox{gray}{{\color{gray}{gray}}}$ ) | 3 (D1)
IO27 ( $\colorbox{yellow}{{\color{yellow}{yellow}}}$ ) | 4 (D2)
IO26 ( $\colorbox{brown}{{\color{brown}{brown}}}$ ) | 5 (D3)
IO9  ( $\colorbox{blue}{{\color{blue}{blue}}}$ ) | 6 (D4)
IO10 ( $\colorbox{purple}{{\color{purple}{purple}}}$ ) | 7 (D5)
IO18 ( $\colorbox{pink}{{\color{pink}{pink}}}$ ) | 8 (D6)
IO23 ( $\colorbox{green}{{\color{green}{green}}}$ ) | 9 (D7)
GND | GND (18-25)
**DSS:** | 
IO19 ( $\colorbox{white}{{\color{white}{white}}}$ ) | 17 (FIFOCLK) (Select Printer_) (PC->DSS)
IO22 ( $\colorbox{gray}{{\color{gray}{gray}}}$ ) | 10 (FIFOFULL) (ACK) (DSS->PC)
**StereoIn1Covox:** | 
IO4 ( $\colorbox{brown}{{\color{brown}{brown}}}$ ) | 1 (Strobe_) (channel select PC->Covox)
&nbsp; | resistor between IO4 and IO25 for external pullup (I have 2.15kohm, 4.7kohm might work too)
**I2S DAC:** |
5V | Vin (use 5V if possible, more stable)
GND | Ground
IO5 | WCLK
IO33 | BLCK
IO32 | DATA
GND | SCK (if GY-PCM5102)
--- | ---
(**Amplifier:**) |
5V | 2-5VDD
GND | Ground
IO25 | Audio In-
IO26 | Audio In+

## Links
- https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
- https://www.weigu.lu/microcontroller/tips_tricks/esp32_tips_tricks/index.html
- https://www.mischianti.org/wp-content/uploads/2020/11/ESP32-DOIT-DEV-KIT-v1-pinout-mischianti.png
- https://www.mischianti.org/wp-content/uploads/2021/07/ESP32-DEV-KIT-DevKitC-v4-pinout-mischianti.png
- https://docs.espressif.com/projects/esp-idf/en/latest/esp32/_images/esp32-pico-kit-v4-pinout.png
- https://europe1.discourse-cdn.com/arduino/optimized/4X/2/3/1/2319cb54d8691ab1d1ba749b7992710217dcacf2_2_500x500.jpeg
- Covox supported games: https://www.mobygames.com/attribute/sheet/attributeId,43/p,2/
- Adafruit Mono 2.5W Class D Audio Amplifier - PAM8302: https://www.adafruit.com/product/2130
- ESP32 DAC: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/dac.html

# Misc stuff

#### ASM
```
C:\Users\lehti\AppData\Local\Arduino15\packages\esp32\tools\xtensa-esp32-elf-gcc\esp-2021r2-patch5-8.4.0\bin\xtensa-esp32-elf-objdump.exe -S C:\Users\lehti\OneDrive\Documents\Arduino\esp32pico_i2s_covox-dss\build\esp32.esp32.esp32\esp32pico_i2s_covox-dss.ino.elf > c:\temp\koe.txt
```

## DosBox
```
[speaker]
lpt_dac = disney, covox, ston1, (or none/off)
```

## Disney Sound Source / DSS

#### ESP32
- Cannot use interrupts, they are too slow. Dedicate another core for bitbanging FIFOCLK signal.
- https://github.com/MacLeod-D/ESp32-Fast-external-IRQs
- Use I2S WCLK for unload buffer interrupt
- DAC doesn't work with 7kHz so double samples to 14kHz

#### Links
- DSS supported games: https://www.mobygames.com/attribute/sheet/attributeId,44
- Miles Sound System (MSS) DSS patch: https://www.vogons.org/viewtopic.php?t=69539
- Disney Sound Source driver for Win3x, Win9x, WinNT 4: http://www.vogonsdrivers.com/getfile.php?fileid=1680&menustate=0
- DSS Programmers Guide: https://archive.org/details/dss-programmers-guide/page/n1/mode/2up
- Reversing the Disney Sound Source: https://www.vogons.org/viewtopic.php?f=62&t=42250&start=140
- DIY DSS: https://www.vogons.org/viewtopic.php?p=474360#p474360
- Vogons Disney Sound Source with FIFO-buffer ICs: https://www.vogons.org/viewtopic.php?p=405296#p405296

#### DSS checks
- Dungeon Master DSS detection: https://www.vogons.org/viewtopic.php?t=40751
- Wolfenstein 3D writes 32 samples to FIFO and checks that FIFOFULL pin activates

#### KDreams
- doesn't detect
- 33 sample interrupts comes trough

## Stereo

- Stereo in 1: Crystal Dream https://www.pouet.net/prod.php?which=463 (LPT pin 1) -> not possible. Pulse is too fast for ESP32. (or is it...)
- https://datasheet.octopart.com/PM7528HP-Analog-Devices-datasheet-11801523.pdf
- http://loboris.eu/ESP32/Xtensa%20Instruction%20Set%20Architecture.pdf
- https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
- 2,15kohm between two gpio (example IO4 and IO26). it works as switchable external pullup for stereo select signal. it is very weak and crucial for operation. 
- Problem in autodetection: after stereo player (iplay, dmp, crystal dreams) DSS-enable signal (LPT pin17) stays up. To get back to normal covox you have to start DSS-program or some other means drop DSS-enable signal down.

## Old garbage

ESP32 DAC output: 25, GND

(to filter DC out put e.g. 10uF capacitor between 25 and out+. to filter noise put 560ohm resistor between 25 and 10uF capacitor and 0.01...0.1uF capacitor between 560ohm and gnd)

(if signal is too high, make resistor divider with two resistor (e.g. 2kohm))

Old pinout was IO12..IO19

NOTICE!
At the moment (7.12.2022, newest ESP32 Adruino IDE board 2.0.5) I cannot get internal DAC to work. Workaround: I use ESP32 Arduino IDE board 1.0.6.

Problems
Commander Keen Dreams doesn't detect this DSS. Why?
Reorganize pins. ESP32 doesn't boot cleanly when connected. Is IO14 and IO15 problem?

ESP32 is 5V tolerant? https://www.qworqs.com/2021/05/19/are-the-esp32-and-esp8266-5v-tolerant-yes-they-officially-are/

And actually there is no 5V from LPT. https://www.epanorama.net/circuits/lptpower.html

Disable WDT: https://www.esp32.com/viewtopic.php?p=62582

```
GPIO_IN_REG GPIO 0-31 input register 0x3FF4403C RO
GPIO_IN1_REG GPIO 32-39 input register 0x3FF44040 RO
```

Note that the I/O GPIO pads are 0-19, 21-23, 25-27, 32-39, while the output GPIOs are 0-19, 21-23, 25-27, 32-33. GPIO pads 34-39 are input-only.

```
// .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT
buf[i] = (0x80<<24) | (s<<8); // IO26 = 0x80, IO25 = sample
```
- ESP32-PICO-KIT with ESP32-PICO-D4 or ESP32 dev board or Wemos D1 mini ESP32

 (for ESP32 internal DAC: Mono 2.5W Class D Audio Amplifier - PAM8302)
