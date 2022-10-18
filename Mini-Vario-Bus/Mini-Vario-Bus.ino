//#define DEBUG
//#define DUAL
// time constants of filter
#define T1 150000.0L
#define T2 200000.0L 

#include <Wire.h>
#include <MS5611.h>             //  https://github.com/nichtgedacht/Arduino-MS5611-Interrupt

#ifndef DEBUG
#include "JetiExBusProtocol.h"  // https://github.com/Sepp62/JetiExBus
JetiExBusProtocol exBus;
#endif

#ifndef DEBUG
enum {
    ID_CLIMB_1 = 1,
    ID_CLIMB_2,
    ID_ALTITUDE_1,
    ID_ALTITUDE_2
};

JETISENSOR_CONST sensors[] PROGMEM = {
    // id            name          unit         data type             precision 0->0, 1->0.0, 2->0.00
    {ID_CLIMB_1, "Climb_1", "m/s", JetiSensor::TYPE_14b, 2},
#ifdef DUAL    
    {ID_CLIMB_2, "Climb_2", "m/s", JetiSensor::TYPE_14b, 2},
#endif
    {ID_ALTITUDE_1, "AltRelat._1", "m", JetiSensor::TYPE_14b, 1},
#ifdef DUAL    
    {ID_ALTITUDE_2, "AltRelat._2", "m", JetiSensor::TYPE_14b, 1},
#endif    
    0                           // end of array
};
#endif //ndef DEBUG

double referencePressure_1 = 0, referencePressure_2 = 0;
double r_altitude_1 = 0, r_altitude0_1 = 0, r_altitude_2 = 0, r_altitude0_2 = 0; 
double climb_1 = 0, climb0_1 = 0, climb_2 = 0, climb0_2 = 0; 
double dyn_alfa_1, dyn_alfa_2, alfa_1, alfa_2, factor;

uint32_t diff_t_A, max_diff_t_A, diff_t_B, max_diff_t_B;
double relativeAltitude_1 = 0, relativeAltitude_2 = 0; 


#ifdef DEBUG
// debug function for checking double or float values
void printDouble( double val, byte precision){
  // prints val with number of decimal places determine by precision
  // precision is a number from 0 to 6 indicating the desired decimial places
  // example: printDouble( 3.1415, 2); // prints 3.14 (two decimal places)

  Serial.print (int(val));  //prints the int part
  if( precision > 0) {
    Serial.print("."); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision -1;
    while(precision--)
       mult *=10;
       
    if(val >= 0)
      frac = (val - int(val)) * mult;
    else
      frac = (int(val)- val ) * mult;
    unsigned long frac1 = frac;
    while( frac1 /= 10 )
      padding--;
    while(  padding--)
      Serial.print("0");
    Serial.print(frac,DEC) ;
  }
}
#endif

void setup () {

    int i;

#ifdef DEBUG
    Serial.begin (115200);
    //Serial.println("Initialize MS5611 Sensor");
#endif

    // Initialize MS5611 sensor(s)
    // first argument oversampling rate pressure, second for temperature
    // if third argument is "true" second sensor with alternative (non default) address becomes active
    // remaining arguments are for the second sensor
    // except the third argument the following call is showing the default values of the lib

#ifdef DUAL      
    while (!ms5611.begin (MS5611_ULTRA_HIGH_RES, MS5611_STANDARD, true, MS5611_ULTRA_HIGH_RES, MS5611_STANDARD)) {
#else
    while (!ms5611.begin (MS5611_ULTRA_HIGH_RES, MS5611_STANDARD)) {
#endif

    // if a single sensor is used, only 2 args needed as in the old version of the lib: 
    // while (!ms5611.begin (MS5611_ULTRA_HIGH_RES, MS5611_STANDARD )) {
    // if these 2 args are as shown they can be ommited also:
    // while (!ms5611.begin ()) {
    
     
#ifdef DEBUG
        Serial.println ("Could not find a valid MS5611 sensor, check wiring!");
#endif
        delay (500);
    }

    // calc alfas from ms5611.delta_t for time constants chosen
    // ms5611.delta_t depends on number of sensors and oversampling rates chosen
    alfa_1 = ms5611.delta_t / ( T1 + ms5611.delta_t );
    alfa_2 = ms5611.delta_t / ( T2 + ms5611.delta_t );

    // calc gain from time constants chosen 
    factor = 1000000 / (T2 - T1) ;

    // warm up
    i = 0;
    while (i < 100) {
        if (ms5611.data_ready) {
            ms5611.getPressure (true, 1);
#ifdef DUAL            
            ms5611.getPressure (true, 2);
#endif            
            i++;
        }
    }

    // get reference
    i = 0;
    while (i < 100) {
        if (ms5611.data_ready) {
            referencePressure_1 += ms5611.getPressure (true, 1);
#ifdef DUAL            
            referencePressure_2 += ms5611.getPressure (true, 2);
#endif            
            i++;
        }
    }
    referencePressure_1 = referencePressure_1 / 100;
#ifdef DUAL    
    referencePressure_2 = referencePressure_2 / 100;
#endif    

#ifndef DEBUG
    exBus.SetDeviceId(0x76, 0x32); // 0x3276
    exBus.Start ("mini_vario", sensors, 0);
#endif

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite( 13, LOW );
}

void loop () {

    if (ms5611.data_ready) {    // flag is interrupt trigggered and reset by ms5611.getPressure  

        long realPressure_1 = ms5611.getPressure (true, 1);
#ifdef DUAL        
        long realPressure_2 = ms5611.getPressure (true, 2);
#endif        
        
        relativeAltitude_1 = ms5611.getAltitude (realPressure_1, referencePressure_1);
#ifdef DUAL        
        relativeAltitude_2 = ms5611.getAltitude (realPressure_2, referencePressure_2);
#endif

/*
 *   dt means sample time interval [s] (delta_t)
 *   T  means time constant of exponential filter [s] (T1, T2)
 *   dT means difference of these Time constants [s] (T2 - T1)
 *   dx means difference of the results of both exponetial filters  
 *   alpha means smoothsness factor ( determines T )
 *   
 *   exponential filter:
 *   output[n] = output[n-1] - alpha * ( output[n-1] - input )
 *   
 *   if alpha == 1 then the output will be equal the input without
 *   any filtering
 *   
 *   relations:
 *   
 *   climb = dx / dT
 *   
 *   alpha = dt / ( T + dt )
 *   T = (dt / alpha) - dt
 * 
 *   dT = T2 - T1
 *   dT = (dt / alpha1) - ( dt / alpha2 ) 
 * 
 */        
        r_altitude0_1 = r_altitude0_1 - alfa_1 * (r_altitude0_1 - relativeAltitude_1);
#ifdef DUAL        
        r_altitude0_2 = r_altitude0_2 - alfa_1 * (r_altitude0_2 - relativeAltitude_2);
#endif        
        
        r_altitude_1 = r_altitude_1 -  alfa_2 * (r_altitude_1 - relativeAltitude_1);
#ifdef DUAL        
        r_altitude_2 = r_altitude_2 -  alfa_2 * (r_altitude_2 - relativeAltitude_2);
#endif
    
        climb0_1 = (r_altitude0_1 - r_altitude_1) * factor;   // Factor is 1000000/dT ( 1/dT as seconds )
#ifdef DUAL        
        climb0_2 = (r_altitude0_2 - r_altitude_2) * factor;   // Factor is 1000000/dT ( 1/dT as seconds )
#endif
        
        // smoothing the climb value by another exponential filter
        // time constant of filter changes dynamically
        // greater speed of change means less filtering.
        // see "Nonlinear Exponential Filter"   
        dyn_alfa_1 = abs( (climb_1 - climb0_1) / 0.8 );
#ifdef DUAL        
        dyn_alfa_2 = abs( (climb_2 - climb0_2) / 0.8 );
#endif        
        if ( dyn_alfa_1 >= 1 ) {
            dyn_alfa_1 = 1;
        }
        
#ifdef DUAL        
        if ( dyn_alfa_2 >= 1 ) {
            dyn_alfa_2 = 1;
        }
#endif                
        climb_1 = climb_1 - dyn_alfa_1 * ( climb_1 - climb0_1 );
#ifdef DUAL        
        climb_2 = climb_2 - dyn_alfa_2 * ( climb_2 - climb0_2 );
#endif        
        
#ifdef DEBUG
        // output for plotter
        Serial.print (climb_1);
        Serial.print ("\t");
        Serial.print (climb_2);     
        Serial.print ("\t");
        Serial.print (r_altitude0_1);
        Serial.print ("\t");
        Serial.println (r_altitude0_2);
        
#else
        exBus.SetSensorValue (ID_CLIMB_1, round ((climb_1) * 100));
#ifdef DUAL             
        exBus.SetSensorValue (ID_CLIMB_2, round ((climb_2) * 100));
#endif            
        exBus.SetSensorValue (ID_ALTITUDE_1, round ((r_altitude0_1) * 10));
#ifdef DUAL            
        exBus.SetSensorValue (ID_ALTITUDE_2, round ((r_altitude0_2) * 10));
#endif
#endif
    }
#ifndef DEBUG    
    exBus.DoJetiExBus();
#endif    
}
