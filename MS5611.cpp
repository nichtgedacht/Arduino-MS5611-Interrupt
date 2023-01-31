/*
MS5611.cpp - Class file for the MS5611 Barometric Pressure & Temperature Sensor Arduino Library.

Version: 4.0.0 by nichtgedacht
modified to be interrupt driven
modified to work with dual sensors
modified to work on Samd21 

Derived from:
-------------------------------------------------------------------------------------------------
Version: 1.0.0
(c) 2014 Korneliusz Jarzebski
www.jarzebski.pl

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

bool MS5611::begin(ms5611_osr_t osr_p_1, ms5611_osr_t osr_t_1, bool dual,
                   ms5611_osr_t osr_p_2, ms5611_osr_t osr_t_2) {
    Wire.begin();

    Wire.setClock(500000L);

    reset(dual);

    delay(100);

    setOversampling(osr_p_1, osr_t_1, dual, osr_p_2, osr_t_2);

    readPROM(dual);

    ms5611.prepareConversion_D2(MS5611_ADDRESS_1);
    if (dual) {
        ms5611.prepareConversion_D2(MS5611_ADDRESS_2);
    }
    data_ready = false;

#if defined (__AVR_ATmega328P__) || (defined __AVR_ATmega328PB__) || defined (__AVR_ATmega32U4__)
    timer1Init(dual);
#elif defined (__SAMD21__)
    TC4_Init(dual);
#endif

    if (dual) {
        dual = true;
    }

    return true;
}

#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328PB__) || defined (__AVR_ATmega32U4__) 
void MS5611::timer1Init(bool dual) {

    uint16_t ct_t, ct_p;

    // timer setup
    cli();                      // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;                  // Counter Register 0

    uint8_t mult = F_CPU / 8000000L;
    if (dual) {

        // use greater conversion times for timer compares
        if (ct_t_1 > ct_t_2) {
            ct_t = ct_t_1;
        } else {
            ct_t = ct_t_2;
        }

        if (ct_p_1 > ct_p_2) {
            ct_p = ct_p_1;
        } else {
            ct_p = ct_p_2;
        }

        OCR1B = ct_t * mult + READ_PREP_COMP_A_DUAL;
        OCR1A = ct_t * mult + ct_p * mult + READ_PREP_COMP_A_DUAL + READ_PREP_COMP_B_DUAL;
        delta_t = ct_t + ct_p + READ_PREP_COMP_A_DUAL + READ_PREP_COMP_B_DUAL;

    } else {

        OCR1B = ct_t_1 * mult + READ_PREP_COMP_A_SINGLE;
        OCR1A = delta_t =
            ct_t_1 * mult + ct_p_1 * mult + READ_PREP_COMP_A_SINGLE + READ_PREP_COMP_B_SINGLE;
        delta_t = ct_t_1 + ct_p_1 + READ_PREP_COMP_A_SINGLE + READ_PREP_COMP_B_SINGLE;
    }

    TCCR1B |= (1 << WGM12);     // CTC mode
    TCCR1B |= (1 << CS11);      // prescaler=8, microsecond tics for 8 MHz MCU 
    TIMSK1 |= (1 << OCIE1A);    // Timer Compare Interrupt activ
    TIMSK1 |= (1 << OCIE1B);    // Timer Compare Interrupt w.o. reset activ
    sei();                      // enable all interrupts
}
#elif defined (__SAMD21__)
void MS5611::TC4_Init(bool dual) {

    uint16_t ct_t, ct_p, cc1_time, cc0_time;

    if (dual) {

        // use greater conversion times for timer compares
        if (ct_t_1 > ct_t_2) {
            ct_t = ct_t_1;
        } else {
            ct_t = ct_t_2;
        }

        if (ct_p_1 > ct_p_2) {
            ct_p = ct_p_1;
        } else {
            ct_p = ct_p_2;
        }

        cc1_time = ct_t + READ_PREP_COMP_A_DUAL;
        cc0_time = ct_t + ct_p + READ_PREP_COMP_A_DUAL + READ_PREP_COMP_B_DUAL;
        delta_t = ct_t + ct_p + READ_PREP_COMP_A_DUAL + READ_PREP_COMP_B_DUAL;

    } else {

        cc1_time = ct_t_1 + READ_PREP_COMP_A_SINGLE;
        cc0_time = delta_t = ct_t_1 + ct_p_1 + READ_PREP_COMP_A_SINGLE + READ_PREP_COMP_B_SINGLE;
        delta_t = ct_t_1 + ct_p_1 + READ_PREP_COMP_A_SINGLE + READ_PREP_COMP_B_SINGLE;
    }    

    /* Setup a generic clock GCLK4 for 16MHz  */
    GCLK->GENDIV.reg = GCLK_GENDIV_DIV(3) |         // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                       GCLK_GENDIV_ID(4);           // Select Generic Clock (GCLK) 4

    GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                        GCLK_GENCTRL_GENEN |        // Enable GCLK4
                        GCLK_GENCTRL_SRC_DFLL48M |  // Set the 48MHz clock source
                        GCLK_GENCTRL_ID(4);         // Select GCLK4
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    /* Feed Clock to TC4 and TC5 */
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |        // Enable GCLK4 to TC4 and TC5
                        GCLK_CLKCTRL_GEN_GCLK4 |    // Select GCLK4
                        GCLK_CLKCTRL_ID_TC4_TC5;    // Feed the GCLK4 to TC4 and TC5

    /* Configure TC4 as 16 Bit counter for 1uS per tick and MPWM mode */
    TC4->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV16 |     // Set prescaler to 16, 48MHz/3/16 = 1MHz (1 us)
                             TC_CTRLA_WAVEGEN_MPWM;         // Put the timer TC4 into match PWM (MPWM) mode.

    /* Setup CC1 and CC0 */
    TC4->COUNT16.CC[1].reg = cc1_time - 1;          // Set the TC4 CC1 register. At this value the first Interrupt occurs 
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);   
    TC4->COUNT16.CC[0].reg = cc0_time - 1;          // Set the TC4 CC0 register. After this value the counter resets
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);


    /* Setup NVIC for TC4 */
    NVIC_SetPriority(TC4_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
    NVIC_EnableIRQ(TC4_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

    /* Enable MC0 and MC1 interrupts */
    TC4->COUNT16.INTENSET.reg = TC_INTENSET_MC0 | TC_INTENSET_MC1;    // Enable MC0 and MC1 interrupts

    /* Start TC4 */
    TC4->COUNT16.CTRLA.bit.ENABLE = 1; 

}    
#endif

// Set oversampling value
void MS5611::setOversampling(ms5611_osr_t osr_p_1, ms5611_osr_t osr_t_1,
                             bool dual, ms5611_osr_t osr_p_2, ms5611_osr_t osr_t_2) {
                                
    switch (osr_p_1) {
    case MS5611_ULTRA_LOW_POWER:
        ct_p_1 = 600;
        break;
    case MS5611_LOW_POWER:
        ct_p_1 = 1170;
        break;
    case MS5611_STANDARD:
        ct_p_1 = 2280;
        break;
    case MS5611_HIGH_RES:
        ct_p_1 = 4540;
        break;
    case MS5611_ULTRA_HIGH_RES:
        ct_p_1 = 9040;
        break;
    }

    uosr_p_1 = osr_p_1;

    switch (osr_t_1) {
    case MS5611_ULTRA_LOW_POWER:
        ct_t_1 = 600;
        break;
    case MS5611_LOW_POWER:
        ct_t_1 = 1170;
        break;
    case MS5611_STANDARD:
        ct_t_1 = 2280;
        break;
    case MS5611_HIGH_RES:
        ct_t_1 = 4540;
        break;
    case MS5611_ULTRA_HIGH_RES:
        ct_t_1 = 9040;
        break;
    }

    uosr_t_1 = osr_t_1;

    if (dual) {

        switch (osr_p_2) {
        case MS5611_ULTRA_LOW_POWER:
            ct_p_2 = 600;
            break;
        case MS5611_LOW_POWER:
            ct_p_2 = 1170;
            break;
        case MS5611_STANDARD:
            ct_p_2 = 2280;
            break;
        case MS5611_HIGH_RES:
            ct_p_2 = 4540;
            break;
        case MS5611_ULTRA_HIGH_RES:
            ct_p_2 = 9040;
            break;
        }

        uosr_p_2 = osr_p_2;

        switch (osr_t_2) {
        case MS5611_ULTRA_LOW_POWER:
            ct_t_2 = 600;
            break;
        case MS5611_LOW_POWER:
            ct_t_2 = 1170;
            break;
        case MS5611_STANDARD:
            ct_t_2 = 2280;
            break;
        case MS5611_HIGH_RES:
            ct_t_2 = 4540;
            break;
        case MS5611_ULTRA_HIGH_RES:
            ct_t_2 = 9040;
            break;
        }

        uosr_t_2 = osr_t_2;

    }
}

void MS5611::reset(bool dual) {

    Wire.beginTransmission(MS5611_ADDRESS_1);

#if ARDUINO >= 100
    Wire.write(MS5611_CMD_RESET);
#else
    Wire.send(MS5611_CMD_RESET);
#endif

    Wire.endTransmission();

    if (dual) {
        Wire.beginTransmission(MS5611_ADDRESS_2);

#if ARDUINO >= 100
        Wire.write(MS5611_CMD_RESET);
#else
        Wire.send(MS5611_CMD_RESET);
#endif

        Wire.endTransmission();
    }
}

void MS5611::readPROM(bool dual) {
    for (uint8_t offset = 0; offset < 6; offset++) {
        fc_1[offset] = readRegister16(MS5611_CMD_READ_PROM + (offset * 2), MS5611_ADDRESS_1);
    }
    if (dual) {
        for (uint8_t offset = 0; offset < 6; offset++) {
            fc_2[offset] = readRegister16(MS5611_CMD_READ_PROM + (offset * 2), MS5611_ADDRESS_2);
        }
    }
}

uint32_t MS5611::readRawTemperature(uint8_t address) {
    return readRegister24(MS5611_CMD_ADC_READ, address);
}

uint32_t MS5611::readRawPressure(uint8_t address) {
    return readRegister24(MS5611_CMD_ADC_READ, address);
}

void MS5611::prepareConversion_D1(uint8_t address) {

    Wire.beginTransmission(address);

    if (address == MS5611_ADDRESS_1) {

#if ARDUINO >= 100
        Wire.write(MS5611_CMD_CONV_D1 + uosr_p_1);
#else
        Wire.send(MS5611_CMD_CONV_D1 + uosr_p_1);
#endif

    } else {

#if ARDUINO >= 100
        Wire.write(MS5611_CMD_CONV_D1 + uosr_p_2);
#else
        Wire.send(MS5611_CMD_CONV_D1 + uosr_p_2);
#endif

    }

    Wire.endTransmission();
}

void MS5611::prepareConversion_D2(uint8_t address) {

    Wire.beginTransmission(address);

    if (address == MS5611_ADDRESS_1) {

#if ARDUINO >= 100
        Wire.write(MS5611_CMD_CONV_D2 + uosr_t_1);
#else
        Wire.send(MS5611_CMD_CONV_D2 + uosr_t_1);
#endif

    } else {

#if ARDUINO >= 100
        Wire.write(MS5611_CMD_CONV_D2 + uosr_t_2);
#else
        Wire.send(MS5611_CMD_CONV_D2 + uosr_t_2);
#endif

    }

    Wire.endTransmission();
}


// using D1 and D2      
int32_t MS5611::getPressure(bool compensation, uint8_t sensor) {

    data_ready = false;         // will be set again by ISR

    int32_t dT;
    int64_t SENS;
    int64_t OFF;
    uint32_t P;


    if (sensor == 1) {

        dT = D2_1 - (uint32_t) fc_1[4] * 256;

        OFF = (int64_t) fc_1[1] * 65536 + (int64_t) fc_1[3] * dT / 128;
        SENS = (int64_t) fc_1[0] * 32768 + (int64_t) fc_1[2] * dT / 256;

    } else {

        dT = D2_2 - (uint32_t) fc_2[4] * 256;

        OFF = (int64_t) fc_2[1] * 65536 + (int64_t) fc_2[3] * dT / 128;
        SENS = (int64_t) fc_2[0] * 32768 + (int64_t) fc_2[2] * dT / 256;

    }

    if (compensation) {

        int32_t TEMP;

        if (sensor == 1) {
            TEMP = 2000 + ((int64_t) dT * fc_1[5]) / 8388608;
        } else {
            TEMP = 2000 + ((int64_t) dT * fc_2[5]) / 8388608;
        }

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

    if (sensor == 1) {
        P = (D1_1 * SENS / 2097152 - OFF) / 32768;
    } else {
        P = (D1_2 * SENS / 2097152 - OFF) / 32768;
    }

    return P;
}

// using D2
double MS5611::getTemperature(bool compensation, uint8_t sensor) {

    int32_t dT;
    int32_t TEMP, TEMP2;

    if (sensor == 1) {

        dT = D2_1 - (uint32_t) fc_1[4] * 256;

        TEMP = 2000 + ((int64_t) dT * fc_1[5]) / 8388608;

    } else {

        dT = D2_2 - (uint32_t) fc_2[4] * 256;

        TEMP = 2000 + ((int64_t) dT * fc_2[5]) / 8388608;

    }

    TEMP2 = 0;

    if (compensation) {
        if (TEMP < 2000) {
            TEMP2 = (dT * dT) / ((uint32_t) 2 << 30);
        }
    }

    TEMP = TEMP - TEMP2;

    return ((double) TEMP / 100);
}

// Calculate altitude from Pressure & Sea level pressure
double MS5611::getAltitude(double pressure, double seaLevelPressure) {
    return (44330.0f * (1.0f - pow((double) pressure / (double) seaLevelPressure, 0.1902949f)));
}

// Calculate sea level from Pressure given on specific altitude
double MS5611::getSeaLevel(double pressure, double altitude) {
    return ((double) pressure / pow(1.0f - ((double) altitude / 44330.0f), 5.255f));
}

// Read 16-bit from register (oops MSB, LSB)
uint16_t MS5611::readRegister16(uint8_t reg, uint8_t address) {
    uint16_t value;
    Wire.beginTransmission(address);
#if ARDUINO >= 100
    Wire.write(reg);
#else
    Wire.send(reg);
#endif
    Wire.endTransmission();

    Wire.requestFrom(address, 2);
#if ARDUINO >= 100
    uint8_t vha = Wire.read();
    uint8_t vla = Wire.read();
#else
    uint8_t vha = Wire.receive();
    uint8_t vla = Wire.receive();
#endif
    Wire.endTransmission();

    value = vha << 8 | vla;

    return value;
}

// Read 24-bit from register (oops XSB, MSB, LSB)
uint32_t MS5611::readRegister24(uint8_t reg, uint8_t address) {
    uint32_t value;
    Wire.beginTransmission(address);
#if ARDUINO >= 100
    Wire.write(reg);
#else
    Wire.send(reg);
#endif
    Wire.endTransmission();

    Wire.requestFrom(address, 3);
#if ARDUINO >= 100
    uint8_t vxa = Wire.read();
    uint8_t vha = Wire.read();
    uint8_t vla = Wire.read();
#else
    uint8_t vxa = Wire.receive();
    uint8_t vha = Wire.receive();
    uint8_t vla = Wire.receive();
#endif
    Wire.endTransmission();

    value = ((int32_t) vxa << 16) | ((int32_t) vha << 8) | vla;

    return value;
}

#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328PB__) || defined (__AVR_ATmega32U4__)
ISR(TIMER1_COMPB_vect) {

//    ms5611.t1 = micros();

    sei();                      // default is disabled but needed for wire lib here

    ms5611.D2_1 = ms5611.readRawTemperature(MS5611_ADDRESS_1);
    if (ms5611.dual) {
        ms5611.D2_2 = ms5611.readRawTemperature(MS5611_ADDRESS_2);
    }
    // prepare for Pressure

    ms5611.prepareConversion_D1(MS5611_ADDRESS_1);
    if (ms5611.dual) {
        ms5611.prepareConversion_D1(MS5611_ADDRESS_2);  // prepareConversion on second sensor (dual == true)
    }
//      ms5611.t2 = micros();
}

ISR(TIMER1_COMPA_vect) {

//      ms5611.t3 = micros();

    sei();                      // default is disabled but needed for wire lib here

    ms5611.D1_1 = ms5611.readRawPressure(MS5611_ADDRESS_1);
    if (ms5611.dual) {
        ms5611.D1_2 = ms5611.readRawPressure(MS5611_ADDRESS_2);
    }
    // prepare for Temperature

    ms5611.prepareConversion_D2(MS5611_ADDRESS_1);
    if (ms5611.dual) {
        ms5611.prepareConversion_D2(MS5611_ADDRESS_2);  // prepareConversion on second sensor (dual == true)
    }

    ms5611.data_ready = true;

//      ms5611.t4 = micros();

}
#elif defined (__SAMD21__)
// Interrupt Service Routine (ISR) for timer TC4
void TC4_Handler() {

    // ms5611.t1 = micros();

    /* Read pressure(s)*/
    if (TC4->COUNT16.INTFLAG.bit.MC0 && TC4->COUNT16.INTENSET.bit.MC0) {
        // digitalWrite(13, LOW);

        ms5611.D1_1 = ms5611.readRawPressure(MS5611_ADDRESS_1);
        if (ms5611.dual) {
            ms5611.D1_2 = ms5611.readRawPressure(MS5611_ADDRESS_2);
        }

        /* Prepare for Temperature(s) conversion */
        ms5611.prepareConversion_D2(MS5611_ADDRESS_1);
        if (ms5611.dual) {
            ms5611.prepareConversion_D2(MS5611_ADDRESS_2);  // prepareConversion on second sensor (dual == true)
        }

        ms5611.data_ready = true;
    }

    if (TC4->COUNT16.INTFLAG.bit.MC1 && TC4->COUNT16.INTENSET.bit.MC1) {
        // digitalWrite(13, HIGH);

        /* Read temperature(s) */
        ms5611.D2_1 = ms5611.readRawTemperature(MS5611_ADDRESS_1);
        if (ms5611.dual) {
            ms5611.D2_2 = ms5611.readRawTemperature(MS5611_ADDRESS_2);
        }

        /* prepare for Pressure(s) conversion */
        ms5611.prepareConversion_D1(MS5611_ADDRESS_1);
        if (ms5611.dual) {
            ms5611.prepareConversion_D1(MS5611_ADDRESS_2);  // prepareConversion on second sensor (dual == true)
        }
    }

    /* clear the interrupt flags */
    TC4->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0 | TC_INTFLAG_MC1;

    // ms5611.t2 = micros();
 }
#endif
