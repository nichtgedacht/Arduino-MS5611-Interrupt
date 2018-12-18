Arduino-MS5611
===============

MS5611 Barometric Pressure & Temperature Sensor Arduino Library

This library use I2C to communicate, 2 pins are required to interface.

Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/czujnik-cisnienia-i-temperatury-ms5611.html

![MS5611](http://www.jarzebski.pl/media/full/publish/2014/05/ms5611-simple.png)

![MS5611](http://www.jarzebski.pl/media/big/publish/2014/05/ms5611-processing.png)

The library has been modified to be interrupt driven
This way the delays within the original code are eliminated
There is plenty of time to implement more sophisticated filters now.
Exact go around time of 20ms with the highest resolution by polling generated flag. 
