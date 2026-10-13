# Hardware and pin assignments

## Components

Microcontroller: ESP32S3 Xiao by Seeed Studio.
- D0 Left Motor PWM
- D1 Right Motor PWM
- D2 Left Motor CW
- D3 Right Motor CW
- D4 = I2C (400MHz) SDA
- D5 = I2C (400MHz) SCL
- D6 = Left Motor CAP via level shifter
- D7 = Right Motor CAP via level shifter
- D10 = Battery Monitor After Voltage Divider

Radio communication using BLE from MCU onboard connection

Drive Motor 2x JGY2430 12V Brushless Worm Gear motor  (12V, 40RPM @ no load, D8x18.8mm shaft)
- Red = 12V+
- Black = GMD
- Yellow = CAP (5V Pulse Feedback to MCU)
- White = PWM Speed Input (3.3V ok) 
- Orange = CW / CCW Input (3.3V ok)
(Left Motor = 1. Right = 2)

Signal Level Shifter (tested to 400MHz)
- 5V   |  3.3V
- GND  |  GND
- H1 (5V SDA) | L1 (3.3V SDA) 
- H2 (5V SCL) | L2 (3.3V SCL) 
- H3 (5V CAPL) | L3 (3.3V CAPL) 
- H4 (5V CAPR) | L4 (3.3V CAPR) 

Line Sensor 8 channel I2C communication @ 400MHz
- SDA (Need to Solder Wire)
- SCL
- GND
- 5V

NFC Reader with FM17550 with I2C communication @ 400kHz
- 3.3V
- GND
- SDA
- SCL

Battery at 12V Li-ion Battery 6000mah (3S2P Config)

Battery Monitor Voltage Divider
- R1 100k, R2 22k. Ratio (22/122)
- Default 12-bit resolution + ADC_ATTEN_DB_11: 0.3.3V = 0-4095
- Ful voltage 12.6V: After Division = 2.68V, ADC = 3539 / 4095
- Safe Cut off 9.5V: After Division = 2.02V, ADC = 2668 / 4095
Voltage Regulator (Battery In 5V out, need an extra schottky diode at output)
