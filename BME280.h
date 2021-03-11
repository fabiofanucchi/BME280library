/**
  @author Fabio Fanucchi - fabiofanucchi5@gmail.com
  @date 05/09/2019
  @updated 17/10/2019
  @version 1.0.23

  @dir "./BME280"
  └> "BME280/BME280.h"
  └> "BME280/BME280.cpp"

  @brief Library for an I2C BME280 sensor that implements:
     - modes: sleep, normal, forced
		 - standby times between measurement in normal mode
		 - IIR filters
		 - over samplings on the 3 measurements

  @see BME280 DS https://www.bosch-sensortec.com/bst/products/all_products/bme280

  @license
    Copyright (C) 2019  Fabio Fanucchi
    This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
    License as published by the Free Software Foundation, either version 3 of the License or any later version.
    This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY, without even the implied
    warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    See the GNU General Public License for more details: https://www.gnu.org/licenses/gpl-3.0.html

  @file "BME280/BME280.h"
*/

#ifndef BME280_h
#define BME280_h

#include <Arduino.h>
#include <Wire.h>

#define BME280_I2CADDRESS 0x76
#define BME280_I2CADDRESS_ALTERNATE 0x77

/**REGISTERS
*/
#define BME280_DIG_T1_LSB_REG 0x88 // ADC compensation
#define BME280_DIG_T1_MSB_REG 0x89
#define BME280_DIG_T2_LSB_REG 0x8A
#define BME280_DIG_T2_MSB_REG 0x8B
#define BME280_DIG_T3_LSB_REG 0x8C
#define BME280_DIG_T3_MSB_REG 0x8D
#define BME280_DIG_P1_LSB_REG 0x8E
#define BME280_DIG_P1_MSB_REG 0x8F
#define BME280_DIG_P2_LSB_REG 0x90
#define BME280_DIG_P2_MSB_REG 0x91
#define BME280_DIG_P3_LSB_REG 0x92
#define BME280_DIG_P3_MSB_REG 0x93
#define BME280_DIG_P4_LSB_REG 0x94
#define BME280_DIG_P4_MSB_REG 0x95
#define BME280_DIG_P5_LSB_REG 0x96
#define BME280_DIG_P5_MSB_REG 0x97
#define BME280_DIG_P6_LSB_REG 0x98
#define BME280_DIG_P6_MSB_REG 0x99
#define BME280_DIG_P7_LSB_REG 0x9A
#define BME280_DIG_P7_MSB_REG 0x9B
#define BME280_DIG_P8_LSB_REG 0x9C
#define BME280_DIG_P8_MSB_REG 0x9D
#define BME280_DIG_P9_LSB_REG 0x9E
#define BME280_DIG_P9_MSB_REG 0x9F
#define BME280_DIG_H1_REG 0xA1
#define BME280_CHIP_ID_REG 0xD0 // chip ID == 0x60
#define BME280_RST_REG 0xE0     // softreset: reset if == 0xB6
#define BME280_DIG_H2_LSB_REG 0xE1
#define BME280_DIG_H2_MSB_REG 0xE2
#define BME280_DIG_H3_REG 0xE3
#define BME280_DIG_H4_MSB_REG 0xE4
#define BME280_DIG_H4_LSB_REG 0xE5
#define BME280_DIG_H5_MSB_REG 0xE6
#define BME280_DIG_H6_REG 0xE7
#define BME280_CTRL_HUMIDITY_REG 0xF2    // ctrl_hum: [2:0] humidity OverSampling
#define BME280_STAT_REG 0xF3             // status: [0] is 1 when sensor reads settings from EEPROM, [3] is 1 while measuring and converting
#define BME280_CTRL_MEAS_REG 0xF4        // ctrl_meas: [1:0] mode, [4:2] pressure OverSampling, [7:5] temperature OverSampling
#define BME280_CONFIG_REG 0xF5           // configuration: [0] SPI enable, [4:2] IIR filter, [7:5] tStandby (only in normal mode)
#define BME280_PRESSURE_MSB_REG 0xF7     // pressure MSB
#define BME280_PRESSURE_LSB_REG 0xF8     // pressure LSB
#define BME280_PRESSURE_XLSB_REG 0xF9    // pressure XLSB
#define BME280_TEMPERATURE_MSB_REG 0xFA  // temperature MSB
#define BME280_TEMPERATURE_LSB_REG 0xFB  // temperature LSB
#define BME280_TEMPERATURE_XLSB_REG 0xFC // temperature XLSB
#define BME280_HUMIDITY_MSB_REG 0xFD     // humidity MSB
#define BME280_HUMIDITY_LSB_REG 0xFE     // humidity LSB

/**@enum sensorMode
   @brief contains modes values
*/
enum sensorMode
{
   MODE_SLEEP = 0b00,
   MODE_FORCED = 0b01, // also 0b10
   MODE_NORMAL = 0b11
};

/**@enum sensortStandby
   @brief contains standby times values
*/
enum sensortStandby
{
   STANDBY_MS_0_5 = 0b000,
   STANDBY_MS_10 = 0b110,
   STANDBY_MS_20 = 0b111,
   STANDBY_MS_62_5 = 0b001,
   STANDBY_MS_125 = 0b010,
   STANDBY_MS_250 = 0b011,
   STANDBY_MS_500 = 0b100,
   STANDBY_MS_1000 = 0b101
};

/**@enum sensorFilter
   @brief contains filters values
*/
enum sensorFilter
{
   FILTER_X0 = 0b000,
   FILTER_X2 = 0b001,
   FILTER_X4 = 0b010,
   FILTER_X8 = 0b011,
   FILTER_X16 = 0b100
};

/**@enum sensorSampling
   @brief contains samplings values
*/
enum sensorSampling
{
   SAMPLING_X0 = 0b000,
   SAMPLING_X1 = 0b001,
   SAMPLING_X2 = 0b010,
   SAMPLING_X4 = 0b011,
   SAMPLING_X8 = 0b100,
   SAMPLING_X16 = 0b101
};

/**@struct BME280Settings
   @brief contains settings of the sensor
   @note settings can be changed before @fn begin()
*/
struct BME280Settings
{
public:
   uint8_t I2CAddress = BME280_I2CADDRESS;

   // settings
   sensorMode mode = MODE_NORMAL;
   sensortStandby tStandby = STANDBY_MS_0_5;
   sensorFilter filter = FILTER_X0;
   sensorSampling pressureOS = SAMPLING_X1;
   sensorSampling temperatureOS = SAMPLING_X1;
   sensorSampling humidityOS = SAMPLING_X1;
};

/**@struct BME280Compensation
   @brief contains ADC compensation values
*/
struct BME280Compensation
{
public:
   uint16_t dig_T1;
   int16_t dig_T2;
   int16_t dig_T3;

   uint16_t dig_P1;
   int16_t dig_P2;
   int16_t dig_P3;
   int16_t dig_P4;
   int16_t dig_P5;
   int16_t dig_P6;
   int16_t dig_P7;
   int16_t dig_P8;
   int16_t dig_P9;

   uint8_t dig_H1;
   int16_t dig_H2;
   uint8_t dig_H3;
   int16_t dig_H4;
   int16_t dig_H5;
   int8_t dig_H6;
};

/**@class BME280
   @brief main sensor class
*/
class BME280
{
public:
   BME280Settings settings;
   BME280Compensation compensation;

   /**@constructor BME280
       @brief object contructor
       @param none
       @return none
    */
   BME280();

   /**@fn begin
       @brief begins Wire, checks sensor connection, resets sensor and writes settings
       @param none
       @return true if sensor is correctly connected, false if not
    */
   bool begin();

   /**@fn begin
       @brief begins Wire, checks sensor connection, resets sensor and writes settings
       @param i2c sda pin, i2c scl pin
       @return true if sensor is correctly connected, false if not
    */
   bool begin(uint8_t, uint8_t);

   /**@fn reset
       @brief resets the sensor
       @param none
       @return none
    */
   void reset();

   /**@fn forcedMeasurement
       @brief tells the sensor to take a measurement (needed only in MODE_FORCED)
       @param none
       @return none
    */
   void forcedMeasurement();

   /**@fn forcedMeasurement
       @brief tells the sensor to take a measurement (needed only in MODE_FORCED)
    	         and writes values on 3 pointers given
       @param pointer to pressure, pointer to temperature, pointer to humidity
       @return none
    */
   void forcedMeasurement(float *, float *, float *);

   /**@fn readPressure
       @brief reads and calculate pressure value
       @param none
       @return pressure value
    */
   float readPressure();

   /**@fn readTemperature
       @brief reads and calculate temperature value
       @param none
       @return temperature value
    */
   float readTemperature();

   /**@fn readHumidity
       @brief reads and calculate humidity value
       @param none
       @return humidity value
    */
   float readHumidity();

private:
   // global variable needed for temperature based pressure and humidity correction
   int32_t t_fine;

   /**@fn readCompensation
       @brief reads and stores in compensation trimming parameters
       @param none
       @return none
    */
   void readCompensation();

   /**@fn checkSensorID
       @brief checks if CHIP_ID register contains 0x60
       @param none
       @return true if CHIP_ID contains 0x60 or false
    */
   bool checkSensorID();

   /**@fn isUpdating
       @brief checks if the sensor is copying NVM data to image registers
       @param none
       @return true when sensor is copying, false when done
    */
   bool isUpdating();

   /**@fn isMeasuring
       @brief checks if the sensor is measuring and converting
       @param none
       @return true when measuring, false when done
    */
   bool isMeasuring();

   /**@fn writeMode
       @brief sets sensor mode (writing the corresponding value in the register)
       @param sensor mode
       @return none
    */
   void writeMode(sensorMode);

   /**@fn writetStandby
       @brief sets sensor standby time (writing the corresponding value in the register)
       @param standby time
       @return none
    */
   void writetStandby(sensortStandby);

   /**@fn writeFilter
       @brief sets sensor IIR filter (writing the corresponding value in the register)
       @param filter
       @return none
    */
   void writeFilter(sensorFilter);

   /**@fn writePressureOS
       @brief sets pressure over sampling (writing the corresponding value in the register)
       @param pressure over sampling
       @return none
    */
   void writePressureOS(sensorSampling);

   /**@fn writeTemperatureOS
       @brief sets temperature over sampling (writing the corresponding value in the register)
       @param temperature over sampling
       @return none
    */
   void writeTemperatureOS(sensorSampling);

   /**@fn writeHumidityOS
       @brief sets humidity over sampling (writing the corresponding value in the register)
       @param humidity over sampling
       @return none
    */
   void writeHumidityOS(sensorSampling);

   /**@fn writeRegister
       @brief writes in the register at the given address the given value
       @param register's address; value to be written
       @return none
    */
   void writeRegister(uint8_t, uint8_t);

   /**@fn readRegister
       @brief reads the value of the register at the given address
       @param register's address
       @return register's value
    */
   uint8_t readRegister(uint8_t);

   /**@fn readRegisters
       @brief reads the value of s given number of registers from the given address
       @param output array, first address, number of registers
       @return none
    */
   void readRegisters(uint8_t *, uint8_t, uint8_t);
};

#endif
