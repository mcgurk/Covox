## old pinout:

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
GND ⚫ | GND (18-25)
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
