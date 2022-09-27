/*
MS5611.h - Header file for the MS5611 Barometric Pressure & Temperature Sensor Arduino Library.

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

#ifndef MS5611_h
#define MS5611_h

#include <Arduino.h>

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define MS5611_ADDRESS_1              (0x77)
#define MS5611_ADDRESS_2              (0x76)

#define MS5611_CMD_ADC_READ           (0x00)
#define MS5611_CMD_RESET              (0x1E)
#define MS5611_CMD_CONV_D1            (0x40)
#define MS5611_CMD_CONV_D2            (0x50)
#define MS5611_CMD_READ_PROM          (0xA2)
#define READ_PREP_DUAL	     		  (1090)
#define READ_PREP_SINGLE			  (840)


#define READ_PREP_COMP_A_SINGLE		(530)
#define READ_PREP_COMP_B_SINGLE		(650)
#define READ_PREP_COMP_A_DUAL       (1080)
#define READ_PREP_COMP_B_DUAL       (1100)

typedef enum {
    MS5611_ULTRA_HIGH_RES = 0x08,
    MS5611_HIGH_RES = 0x06,
    MS5611_STANDARD = 0x04,
    MS5611_LOW_POWER = 0x02,
    MS5611_ULTRA_LOW_POWER = 0x00
} ms5611_osr_t;

class MS5611 {
  public:

    bool begin(ms5611_osr_t osr_p_1 = MS5611_ULTRA_HIGH_RES, ms5611_osr_t osr_t_1 = MS5611_STANDARD,
    bool dual = false, ms5611_osr_t osr_p_2 = MS5611_ULTRA_HIGH_RES, ms5611_osr_t osr_t_2 = MS5611_STANDARD );
    double getTemperature(bool compensation = true, uint8_t sensor = 1);
	int32_t getPressure(bool compensation = true, uint8_t sensor = 1);
	double getAltitude(double pressure, double seaLevelPressure = 101325);
	double getSeaLevel(double pressure, double altitude);
    uint32_t readRawTemperature(uint8_t address);
    uint32_t readRawPressure(uint8_t address);
    void prepareConversion_D1(uint8_t address);
    void prepareConversion_D2(uint8_t address);
    volatile bool data_ready;
    uint16_t delta_t;
	volatile uint32_t D1_1, D2_1, D1_2, D2_2;
	uint8_t n_sensors;
	
	// int32_t t1, t2, t3, t4;

  private:

	uint8_t address;
	uint16_t fc_1[6], fc_2[6];
    uint16_t ct_p_1, ct_t_1, ct_p_2, ct_t_2;
    int32_t TEMP2;
    int64_t OFF2, SENS2;
    uint8_t uosr_p_1, uosr_p_2;
    uint8_t uosr_t_1, uosr_t_2;
    void reset(bool dual);
    void readPROM(bool dual);
    uint16_t readRegister16(uint8_t reg, uint8_t address);
    uint32_t readRegister24(uint8_t reg, uint8_t address);
    void setOversampling(ms5611_osr_t osr_p_1, ms5611_osr_t osr_t_1, bool dual, ms5611_osr_t osr_p_2, ms5611_osr_t osr_t_2 );
    void timer1Init(bool dual);
};

extern MS5611 ms5611;

#endif
