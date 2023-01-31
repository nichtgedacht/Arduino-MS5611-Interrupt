Arduino-MS5611-Interrupt
========================

MS5611 Barometric Pressure & Temperature Sensor Arduino Library

This library use I2C to communicate, 2 pins are required to interface.

This is a modified Version of the well known Library Arduino-MS5611
from Korneliusz Jarzebski https://github.com/jarzebski/Arduino-MS5611

The library has been modified to be interrupt driven.
This way the delays within the original code are eliminated.
There is plenty of time now to implement more sophisticated filters etc.
A generated flag data_ready helps to feed a filter with a minimum of
latency. I.e. 12.5ms spaced samples if resolution is MS5611_ULTRA_HIGH_RES
for pressure and MS5611_STANDARD for temperature. A variable delta_t
is assigned by the library. Wire clock for I2C is set to 300 kHz
which works reliable with this sensor.

Update:
The libray supports 2 sensors now. For details please read comments in example Mini-Vario.
I2C speed is increased to 500 kHz. For a single Sensor 12.5ms spaced samples are kept.
If working with 2 sensors the samples are 13.5ms spaced.
This applies to a Pro Mini @8Mhz 

Update:
The library runs on Samd21 now. It uses Timer TC4 there.
