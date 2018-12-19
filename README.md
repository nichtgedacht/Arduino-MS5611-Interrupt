Arduino-MS5611-Interrupt
========================

MS5611 Barometric Pressure & Temperature Sensor Arduino Library

This library use I2C to communicate, 2 pins are required to interface.

This is a modified Version of the well known Library Arduino-MS5611
from Korneliusz Jarzebski https://github.com/jarzebski/Arduino-MS5611

The library has been modified to be interrupt driven.
This way the delays within the original code are eliminated.
There is plenty of time to implement more sophisticated filters now.
A generated flag helps to feed a filter with a minimum of latency.
I.e. 20ms spaced Samples for MS5611_ULTRA_HIGH_RES
