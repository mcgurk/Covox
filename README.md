
# McGurk-Covox/DSS/StereoOn1-System (McDSS)
Parallel/LPT-port soundcard using microcontroller. Supports Covox Speech Thing/Intersound MDO, Disney Sound Source (DSS) and Stereo-on-1 DAC.

## Parts

- ESP32-PICO-KIT (if other ESP32 is used, modify pins)
- I2S DAC (e.g. PCM5102A/GY-PCM5102 or I2S DAC amplifier MAX98357A)

## New wiring (ESP-IDF 5.1.1)

ESP32 | LPT (D25)
--- | ---
**Covox:** |
IO18 🟤 | 2 (D0)
IO19 🟠 | 3 (D1)
IO27 🔵 | 4 (D2)
IO21 🟢 | 5 (D3)
IO22 🟡 | 6 (D4)
IO23 🔴 | 7 (D5)
IO13 🔘 | 8 (D6)
IO14 🟣 | 9 (D7)
GND ⚫ | GND (18-25)
**DSS:** | 
IO9 ⚪ | 17 (FIFOCLK) (Select Printer_) (PC->DSS)
IO10 ⚫ | 10 (FIFOFULL) (ACK) (DSS->PC)
**Stereo-on-1:** | 
IO4 ⚪ | 1 (Strobe_) (channel select PC->Covox)
&nbsp; | resistor between IO4 and 5V for external pullup (I have 2.15kohm, 4.7kohm might work too)
**ESP32:** | **I2S DAC:**
5V 🔴 | Vin (use 5V if possible, more stable)
GND ⚫ | Ground
IO25 🟢 | DATA
IO32 🟡 | BLCK
IO26 🟤 | WCLK (must be 0-31)
GND | SCK (if GY-PCM5102)

\* = IO9 and IO10 is only usable with ESP32-PICO-KIT (ESP32-PICO-D4) (with ESP32-WROOM-32 use 16 and 17)

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
- Xtensa® Instruction Set Architecture (ISA) Reference Manual: https://0x04.net/~mwk/doc/xtensa.pdf
- https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2s.html
- PCM5102: https://www.ti.com/lit/ds/symlink/pcm5100a.pdf
- DSS/Covox exclusive games? https://www.vogons.org/viewtopic.php?t=57885
- LPT: https://www.epanorama.net/circuits/lptpower.html
- LPT: https://www.edaboard.com/threads/parallel-port-max-current-sink-output.49356/

## Covox Speech Thing / Intersound MDO / LPT DAC
- https://en.wikipedia.org/wiki/Covox_Speech_Thing
- https://www.vgmpf.com/Wiki/index.php?title=Intersound_MDO
- Covox supported games: https://www.mobygames.com/attributes/attribute/43/
- Intersound MDO supported games: https://www.mobygames.com/attributes/attribute/1860/
  
## Disney Sound Source (DSS)

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
- Wolfenstein 3D: writes 32 samples to FIFO and checks that FIFOFULL pin activates
- Commander Keen in Keen Dreams (KDreams): v1.0 detects DSS. 33 sample interrupts comes trough. Notice: DSS is not actually supported at all in the game.

## Stereo-on-1
- 2,15kohm between gpio and 5V. Channelselect signal is very fast (weak) and pullup is crucial for operation. 
- Crystal Dream by Triton: https://www.pouet.net/prod.php?which=463 (LPT pin 1)
- Crystal Dream 2 by Triton: https://www.pouet.net/prod.php?which=462
- Inertia Player 1.22: https://www.pouet.net/prod.php?which=29208
- Dual Module Player 3.01: https://files.scene.org/browse/resources/music/players/
- Fast Tracker II (2.09)
- Galaxy Music Player (GLX) v2.12 by Trial: https://www.pouet.net/prod.php?which=73182 (I couldn't get this to work in stereo)
- https://datasheet.octopart.com/PM7528HP-Analog-Devices-datasheet-11801523.pdf
- http://loboris.eu/ESP32/Xtensa%20Instruction%20Set%20Architecture.pdf
- https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
- Problem in autodetection: after stereo player (iplay, dmp, crystal dreams) DSS-enable signal (LPT pin17) stays up. To get back to normal covox you have to start DSS-program or some other means drop DSS-enable signal down.
- Covox in Stereo on one LPT Port - LPTSND X2 (Prototype): https://www.vogons.org/viewtopic.php?t=83891
- Problem with ESP32 GPIO reading: https://github.com/espressif/arduino-esp32/issues/4172

# Misc stuff

### DOSBox
```
[speaker]
disney=true
```

### Dual Module Player (DMP)
```
dmp -c6 -p378 -s44100 -q song.mod # covox
dmp -c12 -p378 -s30000 -q song.mod # stereo-in-one
or
SET DMP=-c12 -p378 -s30000 -q
```

### FastDoom
- https://github.com/viti95/FastDoom
- You need wad file from shareware or retail version
- To get music (with fx) from Covox/DSS, you need music as raw files. There is OGG-files in Steam version, but you may find some Doom ogg-files with google also. You see filenames and how to convert ogg files to raw files from this script: https://github.com/viti95/FastDoom/blob/master/SCRIPTS/PCMconvert/convert.sh
- PCM Music format is unsigned 8-bit PCM, and supports 11025, 22050 or 44100 Hz frequencies. 
- With Covox: Convert ogg to raw in same samplerate as you are selected from fdsetup. Notice that this also affects to minimum memory requirements (whole raw-file must fit to memory at once)



### Create utility to get LPT pin 17 (fifoclk / dss power on) to low with DOS debug-command
Press enter after every line (also when there is empty line).
```
C:\>debug
n pin17.com         ; filename
a100                ; assemble to address :0100
mov dx,37a          ; 37Ah = LPT control pins port
in al,dx            ; read current state from port
or al,8             ; bit 4 high -> pin 17 low ("and al,F7" would get it to high)
out dx,al           ; write new state to port
ret                 ; return to DOS
                    ; end assembler mode with empty line
rcx                 ; give how many bytes to write in cx register
8                   ; 8 bytes to write
w                   ; write file
q                   ; quit
```
