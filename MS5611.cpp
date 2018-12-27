/*
MS5611.cpp - Class file for the MS5611 Barometric Pressure & Temperature Sensor Arduino Library.

Version: 1.0.0
(c) 2014 Korneliusz Jarzebski
www.jarzebski.pl

Version: 2.1.0
modified to be interrupt driven by nichtgedacht 

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include <math.h>

#include <MS5611.h>

MS5611 ms5611;

bool MS5611::begin (ms5611_osr_t osr_p, ms5611_osr_t osr_t) {
    Wire.begin ();

    Wire.setClock (300000L);

    reset ();

    delay (100);

    setOversampling (osr_p, osr_t);

    readPROM ();

    ms5611.prepareConversion_D2 ();
    data_ready = false;

    timer1Init ();

    return true;
}

void
MS5611::timer1Init (void) {

    // timer setup
    cli ();                     // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;                  // Counter Register 0

    uint8_t mult = F_CPU / 8000000;
    OCR1B = ct_t * mult + 590;  // ct_x = tics @8MHz, read + prepare_next < 590 us each
    OCR1A = delta_t = ct_t * mult + 590 + ct_p * mult + 590;

    TCCR1B |= (1 << WGM12);     // CTC mode
    TCCR1B |= (1 << CS11);      // prescaler=8, microsecond tics for 8 MHz MCU 
    TIMSK1 |= (1 << OCIE1A);    // Timer Compare Interrupt activ
    TIMSK1 |= (1 << OCIE1B);    // Timer Compare Interrupt w.o. reset activ
    sei ();                     // enable all interrupts
}

// Set oversampling value
void
MS5611::setOversampling (ms5611_osr_t osr_p, ms5611_osr_t osr_t) {
    switch (osr_p) {
    case MS5611_ULTRA_LOW_POWER:
        ct_p = 600;
        break;
    case MS5611_LOW_POWER:
        ct_p = 1170;
        break;
    case MS5611_STANDARD:
        ct_p = 2280;
        break;
    case MS5611_HIGH_RES:
        ct_p = 4540;
        break;
    case MS5611_ULTRA_HIGH_RES:
        ct_p = 9040;
        break;
    }

    uosr_p = osr_p;

    switch (osr_t) {
    case MS5611_ULTRA_LOW_POWER:
        ct_t = 600;
        break;
    case MS5611_LOW_POWER:
        ct_t = 1170;
        break;
    case MS5611_STANDARD:
        ct_t = 2280;
        break;
    case MS5611_HIGH_RES:
        ct_t = 4540;
        break;
    case MS5611_ULTRA_HIGH_RES:
        ct_t = 9040;
        break;
    }

    uosr_t = osr_t;
}

void
MS5611::reset (void) {

    Wire.beginTransmission (MS5611_ADDRESS);

#if ARDUINO >= 100
    Wire.write (MS5611_CMD_RESET);
#else
    Wire.send (MS5611_CMD_RESET);
#endif

    Wire.endTransmission ();
}

void
MS5611::readPROM (void) {
    for (uint8_t offset = 0; offset < 6; offset++) {
        fc[offset] = readRegister16 (MS5611_CMD_READ_PROM + (offset * 2));
    }
}

void
MS5611::readRawTemperature (void) {
    D2 = readRegister24 (MS5611_CMD_ADC_READ);
}

void
MS5611::readRawPressure (void) {
    D1 = readRegister24 (MS5611_CMD_ADC_READ);
}

void
MS5611::prepareConversion_D1 (void) {

    Wire.beginTransmission (MS5611_ADDRESS);

#if ARDUINO >= 100
    Wire.write (MS5611_CMD_CONV_D1 + uosr_p);
#else
    Wire.send (MS5611_CMD_CONV_D1 + uosr_p);
#endif

    Wire.endTransmission ();
}

void
MS5611::prepareConversion_D2 (void) {

    Wire.beginTransmission (MS5611_ADDRESS);

#if ARDUINO >= 100
    Wire.write (MS5611_CMD_CONV_D2 + uosr_t);
#else
    Wire.send (MS5611_CMD_CONV_D2 + uosr_t);
#endif

    Wire.endTransmission ();
}


// using D1 and D2      
int32_t MS5611::getPressure (bool compensation) {
    data_ready = false;         // will be set again by ISR

    int32_t
        dT = D2 - (uint32_t) fc[4] * 256;

    int64_t
        OFF = (int64_t) fc[1] * 65536 + (int64_t) fc[3] * dT / 128;
    int64_t
        SENS = (int64_t) fc[0] * 32768 + (int64_t) fc[2] * dT / 256;

    if (compensation) {
        int32_t
            TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

        OFF2 = 0;
        SENS2 = 0;

        if (TEMP < 2000) {
            OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
            SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
        }

        if (TEMP < -1500) {
            OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
            SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
        }

        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    uint32_t
        P = (D1 * SENS / 2097152 - OFF) / 32768;

    return P;
}

// using D2
double
MS5611::getTemperature (bool compensation) {

    int32_t dT = D2 - (uint32_t) fc[4] * 256;

    int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

    TEMP2 = 0;

    if (compensation) {
        if (TEMP < 2000) {
            TEMP2 = (dT * dT) / (2 << 30);
        }
    }

    TEMP = TEMP - TEMP2;

    return ((double) TEMP / 100);
}

// Calculate altitude from Pressure & Sea level pressure
double
MS5611::getAltitude (double pressure, double seaLevelPressure) {
    return (44330.0f *
            (1.0f -
             pow ((double) pressure / (double) seaLevelPressure,
                  0.1902949f)));
}

// Calculate sea level from Pressure given on specific altitude
double
MS5611::getSeaLevel (double pressure, double altitude) {
    return ((double) pressure /
            pow (1.0f - ((double) altitude / 44330.0f), 5.255f));
}

// Read 16-bit from register (oops MSB, LSB)
uint16_t MS5611::readRegister16 (uint8_t reg) {
    uint16_t
        value;
    Wire.beginTransmission (MS5611_ADDRESS);
#if ARDUINO >= 100
    Wire.write (reg);
#else
    Wire.send (reg);
#endif
    Wire.endTransmission ();

    Wire.beginTransmission (MS5611_ADDRESS);
    Wire.requestFrom (MS5611_ADDRESS, 2);
    while (!Wire.available ()) {
    };
#if ARDUINO >= 100
    uint8_t
        vha = Wire.read ();
    uint8_t
        vla = Wire.read ();
#else
    uint8_t
        vha = Wire.receive ();
    uint8_t
        vla = Wire.receive ();
#endif /* ; */
    Wire.endTransmission ();

    value = vha << 8 | vla;

    return value;
}

// Read 24-bit from register (oops XSB, MSB, LSB)
uint32_t MS5611::readRegister24 (uint8_t reg) {
    uint32_t
        value;
    Wire.beginTransmission (MS5611_ADDRESS);
#if ARDUINO >= 100
    Wire.write (reg);
#else
    Wire.send (reg);
#endif
    Wire.endTransmission ();

    Wire.beginTransmission (MS5611_ADDRESS);
    Wire.requestFrom (MS5611_ADDRESS, 3);
    while (!Wire.available ()) {
    };
#if ARDUINO >= 100
    uint8_t
        vxa = Wire.read ();
    uint8_t
        vha = Wire.read ();
    uint8_t
        vla = Wire.read ();
#else
    uint8_t
        vxa = Wire.receive ();
    uint8_t
        vha = Wire.receive ();
    uint8_t
        vla = Wire.receive ();
#endif /* ; */
    Wire.endTransmission ();

    value = ((int32_t) vxa << 16) | ((int32_t) vha << 8) | vla;

    return value;
}

ISR (TIMER1_COMPB_vect) {

//      ms5611.t1 = micros();

    sei ();                     // default is disabled but needed for wire lib here

    ms5611.readRawTemperature ();       // assigned to D2

    ms5611.prepareConversion_D1 ();     // prepare for Pressure

//      ms5611.t2 = micros();
}

ISR (TIMER1_COMPA_vect) {

    sei ();                     // default is disabled but needed for wire lib here

    ms5611.readRawPressure ();  // assigned to D1

    ms5611.prepareConversion_D2 ();     // prepare for Temperature

    ms5611.data_ready = true;

}
