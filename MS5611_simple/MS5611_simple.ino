/*                                                                                                                                                                                                  
  MS5611 Barometric Pressure & Temperature Sensor. Simple Example
  Library derived from https://github.com/jarzebski/Arduino-MS5611
*/

#define DEBUG

#include <Wire.h>
#include <MS5611.h>                          


double referencePressure;
int i;

void setup()
{
 
  Serial.begin(9600);
  Serial.println("Initialize MS5611 Sensor");
  
  // Initialize MS5611 sensor
  while(!ms5611.begin(MS5611_ULTRA_HIGH_RES))
  {
    Serial.println("Could not find a valid MS5611 sensor, check wiring!");
    delay(500);
  }

  // warm up
  i = 0;
  while ( i<100 ) {
    if ( ms5611.data_ready ) {
      ms5611.getPressure(true); 
      i++;
    }
  }

  // get reference
  i = 0;
  while ( i<100 ) {
    if ( ms5611.data_ready ) {
       referencePressure += ms5611.getPressure(true);
       i++;
    }
  }
  referencePressure = referencePressure / 100;

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{

  if ( ms5611.data_ready ) { // flag is interrupt trigggered (timer1) and reset by ms5611.getPressure  

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // 20 ms on, 20 ms off if MS5611_ULTRA_HIGH_RES 
  
    long realPressure = ms5611.getPressure(true);
    double relativeAltitude = ms5611.getAltitude(realPressure, referencePressure);

    Serial.println(relativeAltitude);
    
  }
}

