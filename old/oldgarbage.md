## Old garbage

### ASM
```
C:\Users\lehti\AppData\Local\Arduino15\packages\esp32\tools\xtensa-esp32-elf-gcc\esp-2021r2-patch5-8.4.0\bin\xtensa-esp32-elf-objdump.exe -S C:\Users\lehti\OneDrive\Documents\Arduino\esp32pico_i2s_covox-dss\build\esp32.esp32.esp32\esp32pico_i2s_covox-dss.ino.elf > c:\temp\koe.txt
```


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

 ```
[speaker]
lpt_dac = disney, covox, ston1, (or none/off)
```

 ESP32 | amplifier
--- | ---
(**Amplifier:**) |
5V | 2-5VDD
GND | Ground
IO25 | Audio In-
IO26 | Audio In+

https://emojipedia.org/
🔴🟢🟤🟡⚪🔵🟣🟠🌸🐺⚫
