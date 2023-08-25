## old pinout:

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
