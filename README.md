
# Covox, Disney Sound Source (DSS), Stereo in 1
Parallel port / LPT soundcard with microcontroller. Supports Covox Speech Thing (aka Intersound MDO), Disney Sound Source (DSS) and Stereo-in-1 DAC.

## Parts

- ESP32-PICO-KIT (if other ESP32 is used, modify pins)
- I2S DAC (e.g. PCM5102A/GY-PCM5102 or I2S DAC amplifier MAX98357A)

## Wiring

ESP32 | LPT (D25)
--- | ---
**Covox:** |
IO13 ⚪ | 2 (D0)
IO14 🐺 | 3 (D1)
IO27 🟡 | 4 (D2)
IO26 🟤 | 5 (D3)
IO9* 🔵 | 6 (D4)
IO10* 🟣 | 7 (D5)
IO18 🌸 | 8 (D6)
IO23 🟢 | 9 (D7)
GND | GND (18-25)
**DSS:** | 
IO19 ⚪ | 17 (FIFOCLK) (Select Printer_) (PC->DSS)
IO22 🐺 | 10 (FIFOFULL) (ACK) (DSS->PC)
**Stereo-In-1:** | 
IO4 🟤 | 1 (Strobe_) (channel select PC->Covox)
&nbsp; | resistor between IO4 and 5V for external pullup (I have 2.15kohm, 4.7kohm might work too)
**ESP32:** | **I2S DAC:**
5V 🔴 | Vin (use 5V if possible, more stable)
GND ⚫ | Ground
IO5 🟤 | WCLK
IO33 🟡 | BLCK
IO32 🟢 | DATA
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
- Covox supported games: https://www.mobygames.com/attributes/attribute/43/
- DSS/Covox exclusive games? https://www.vogons.org/viewtopic.php?t=57885
- LPT: https://www.epanorama.net/circuits/lptpower.html
- LPT: https://www.edaboard.com/threads/parallel-port-max-current-sink-output.49356/


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
- Wolfenstein 3D: writes 32 samples to FIFO and checks that FIFOFULL pin activates
- Commander Keen in Keen Dreams (KDreams): v1.0 detects DSS. 33 sample interrupts comes trough. Notice: DSS is not actually supported at all in the game.

## Stereo-in-1
- 2,15kohm between gpio and 5V. Channelselect signal is very fast (weak) and pullup is crucial for operation. 
- Stereo in 1: Crystal Dream https://www.pouet.net/prod.php?which=463 (LPT pin 1)
- https://datasheet.octopart.com/PM7528HP-Analog-Devices-datasheet-11801523.pdf
- http://loboris.eu/ESP32/Xtensa%20Instruction%20Set%20Architecture.pdf
- https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
- Problem in autodetection: after stereo player (iplay, dmp, crystal dreams) DSS-enable signal (LPT pin17) stays up. To get back to normal covox you have to start DSS-program or some other means drop DSS-enable signal down.

#### Solve to Compaq Contura 430C (486, 100MHz) problem with stereo-in-1 (random crackling)
- https://www.retrospace.net/download/Compaq%20LTE%20Elite%204-75CX%20Drivers%20and%20Utilities/
  - sp1630.exe, "EPP Support Utility Version 2.00 Rev. A", eppbios.sys (copy EPPBIOS.SYS to C:\CPQDOS\)
  - sp2158.exe, "Parallel Port Configuration 1.00 A", setport.exe (copy SETPORT.EXE to C:\CPQDOS\)

config.sys:
```
DEVICEHIGH=C:\CPQDOS\EPPBIOS.SYS
```
autoexec.bat:
```
C:\CPQDOS\SETPORT.EXE 3
```

# Misc stuff

### DOSBox
```
[speaker]
disney=true
```

### DMP
```
dmp -c6 -p378 -s44100 -q song.mod # covox
dmp -c12 -p378 -s30000 -q song.mod # stereo-in-one
```

### Create utility to get LPT pin 17 (fifoclk) to low with DOS debug-command
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
