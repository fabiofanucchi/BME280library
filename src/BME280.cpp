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

  @file "BME280/BME280.cpp"
*/

#include"BME280.h"

/**@constructor BME280
   @brief object contructor
   @param none
   @return none
*/
BME280::BME280() {
}

/**@fn begin
   @brief begins Wire, checks sensor connection, resets sensor and writes settings
   @param none
   @return true if sensor is correctly connected, false if not
*/
bool BME280::begin() {
  // Wire begin
  Wire.begin();
  // check if sensor is connected, if not try the alternative address
  if (!checkSensorID()) {
    settings.I2CAddress = BME280_I2CADDRESS_ALTERNATE;
    // if also the alternative address is unavailable return false
    if (!checkSensorID()) return false;
  }
  // reset the sensor
  reset();
  // wait while sensor is updating
  while (isUpdating()) delay(1);
  // read compensation coefficients
  readCompensation();
  // put in sleep mode(needed to write values)
  writeMode(MODE_SLEEP);
  // write settings
  writetStandby(settings.tStandby);
  writeFilter(settings.filter);
  writePressureOS(settings.pressureOS);
  writeTemperatureOS(settings.temperatureOS);
  writeHumidityOS(settings.humidityOS);
  // write user defined mode
  writeMode(settings.mode);
  // succes!
  return true;
}

/**@fn reset
   @brief resets the sensor
   @param none
   @return none
*/
void BME280::reset() {
  // write the reset value in the reset register
  writeRegister(BME280_RST_REG, 0xB6);
}

/**@fn forcedMeasurement
   @brief tells the sensor to take a measurement (needed only in MODE_FORCED)
   @param none
   @return none
*/
void BME280::forcedMeasurement() {
  // set to forced mode (tell the sensor to take a measurement!)
  writeMode(MODE_FORCED);
  // wait while measuring and converting
  while (isMeasuring()) delay(1);
}

/**@fn forcedMeasurement
   @brief tells the sensor to take a measurement (needed only in MODE_FORCED)
          and writes values on 3 pointers given
   @param pointer to pressure, pointer to temperature, pointer to humidity
   @return none
*/
void BME280::forcedMeasurement(float *pressure, float *temperature, float *humidity) {
  // take a measurement
  forcedMeasurement();
  // read values
  *pressure = readPressure();
  *temperature = readTemperature();
  *humidity = readHumidity();
}

/**@fn readPressure
   @brief reads and calculate pressure value
   @param none
   @return pressure value
*/
float BME280::readPressure() {
  // read temperature first to get t_fine
  readTemperature();
  // read raw values from registers and compensate them (@see DS)
  uint8_t buffer[3];
  readRegisters(buffer, BME280_PRESSURE_MSB_REG, 3);
  int32_t adc_P = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
  int64_t var1, var2, p_acc;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)compensation.dig_P6;
  var2 = var2 + ((var1 * (int64_t)compensation.dig_P5) << 17);
  var2 = var2 + (((int64_t)compensation.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)compensation.dig_P3) >> 8) + ((var1 * (int64_t)compensation.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)compensation.dig_P1) >> 33;
  if (var1 == 0) return 0; // avoid exception caused by division by 0
  p_acc = 1048576 - adc_P;
  p_acc = (((p_acc << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)compensation.dig_P9) * (p_acc >> 13) * (p_acc >> 13)) >> 25;
  var2 = (((int64_t)compensation.dig_P8) * p_acc) >> 19;
  p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)compensation.dig_P7) << 4);
  return (float)p_acc / 25600.0;
}

/**@fn readTemperature
   @brief reads and calculate temperature value
   @param none
   @return temperature value
*/
float BME280::readTemperature() {
  // read raw values from registers and compensate them (@see DS)
  uint8_t buffer[3];
  readRegisters(buffer, BME280_TEMPERATURE_MSB_REG, 3);
  int32_t adc_T = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
  int64_t var1, var2;
  var1 = ((((adc_T >> 3) - ((int32_t)compensation.dig_T1 << 1))) * ((int32_t)compensation.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)compensation.dig_T1)) * ((adc_T >> 4) - ((int32_t)compensation.dig_T1))) >> 12)
          * ((int32_t)compensation.dig_T3)) >> 14;
  t_fine = var1 + var2;
  float ret = (t_fine * 5 + 128) >> 8;
  return ret / 100.0;
}

/**@fn readHumidity
   @brief reads and calculate humidity value
   @param none
   @return humidity value
*/
float BME280::readHumidity() {
  // read temperature first to get t_fine
  readTemperature();
  // read raw values from registers and compensate them (@see DS)
  uint8_t buffer[2];
  readRegisters(buffer, BME280_HUMIDITY_MSB_REG, 2);
  int32_t adc_H = ((uint32_t)buffer[0] << 8) | ((uint32_t)buffer[1]);
  int32_t var1;
  var1 = (t_fine - ((int32_t)76800));
  var1 = (((((adc_H << 14) - (((int32_t)compensation.dig_H4) << 20) - (((int32_t)compensation.dig_H5) * var1))
            + ((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)compensation.dig_H6)) >> 10) * (((var1 * ((int32_t)compensation.dig_H3)) >> 11)
                                          + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)compensation.dig_H2) + 8192) >> 14));
  var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)compensation.dig_H1)) >> 4));
  var1 = (var1 < 0 ? 0 : var1);
  var1 = (var1 > 419430400 ? 419430400 : var1);
  return (float)(var1 >> 12) / 1024.0;
}

/**@fn readCompensation
   @brief reads and stores in compensation trimming parameters
   @param none
   @return none
*/
void BME280::readCompensation() {
  compensation.dig_T1 = ((uint16_t)((readRegister(BME280_DIG_T1_MSB_REG) << 8) + readRegister(BME280_DIG_T1_LSB_REG)));
  compensation.dig_T2 = ((int16_t)((readRegister(BME280_DIG_T2_MSB_REG) << 8) + readRegister(BME280_DIG_T2_LSB_REG)));
  compensation.dig_T3 = ((int16_t)((readRegister(BME280_DIG_T3_MSB_REG) << 8) + readRegister(BME280_DIG_T3_LSB_REG)));

  compensation.dig_P1 = ((uint16_t)((readRegister(BME280_DIG_P1_MSB_REG) << 8) + readRegister(BME280_DIG_P1_LSB_REG)));
  compensation.dig_P2 = ((int16_t)((readRegister(BME280_DIG_P2_MSB_REG) << 8) + readRegister(BME280_DIG_P2_LSB_REG)));
  compensation.dig_P3 = ((int16_t)((readRegister(BME280_DIG_P3_MSB_REG) << 8) + readRegister(BME280_DIG_P3_LSB_REG)));
  compensation.dig_P4 = ((int16_t)((readRegister(BME280_DIG_P4_MSB_REG) << 8) + readRegister(BME280_DIG_P4_LSB_REG)));
  compensation.dig_P5 = ((int16_t)((readRegister(BME280_DIG_P5_MSB_REG) << 8) + readRegister(BME280_DIG_P5_LSB_REG)));
  compensation.dig_P6 = ((int16_t)((readRegister(BME280_DIG_P6_MSB_REG) << 8) + readRegister(BME280_DIG_P6_LSB_REG)));
  compensation.dig_P7 = ((int16_t)((readRegister(BME280_DIG_P7_MSB_REG) << 8) + readRegister(BME280_DIG_P7_LSB_REG)));
  compensation.dig_P8 = ((int16_t)((readRegister(BME280_DIG_P8_MSB_REG) << 8) + readRegister(BME280_DIG_P8_LSB_REG)));
  compensation.dig_P9 = ((int16_t)((readRegister(BME280_DIG_P9_MSB_REG) << 8) + readRegister(BME280_DIG_P9_LSB_REG)));

  compensation.dig_H1 = ((uint8_t)(readRegister(BME280_DIG_H1_REG)));
  compensation.dig_H2 = ((int16_t)((readRegister(BME280_DIG_H2_MSB_REG) << 8) + readRegister(BME280_DIG_H2_LSB_REG)));
  compensation.dig_H3 = ((uint8_t)(readRegister(BME280_DIG_H3_REG)));
  compensation.dig_H4 = ((int16_t)((readRegister(BME280_DIG_H4_MSB_REG) << 4) + (readRegister(BME280_DIG_H4_LSB_REG) & 0x0F)));
  compensation.dig_H5 = ((int16_t)((readRegister(BME280_DIG_H5_MSB_REG) << 4) + ((readRegister(BME280_DIG_H4_LSB_REG) >> 4) & 0x0F)));
  compensation.dig_H6 = ((int8_t)readRegister(BME280_DIG_H6_REG));
}

/**@fn checkSensorID
   @brief checks if CHIP_ID register contains 0x60
   @param none
   @return true if CHIP_ID contains 0x60 or false
*/
bool BME280::checkSensorID() {
  return (readRegister(BME280_CHIP_ID_REG) == 0x60);
}

/**@fn isUpdating
   @brief checks if the sensor is copying NVM data to image registers
   @param none
   @return true when sensor is copying, false when the copying is done
*/
bool BME280::isUpdating() {
  uint8_t stat = readRegister(BME280_STAT_REG);
  return (stat & 1 << 0);
}

/**@fn isMeasuring
   @brief checks if the sensor is measuring and converting
   @param none
   @return true when measuring, false when done
*/
bool BME280::isMeasuring() {
  uint8_t stat = readRegister(BME280_STAT_REG);
  return (stat & (1 << 3));
}

/**@fn writeMode
   @brief sets sensor mode (writing the corresponding value in the register)
   @param sensor mode
   @return none
*/
void BME280::writeMode(sensorMode mode) {
  uint8_t data = readRegister(BME280_CTRL_MEAS_REG);
  data &= ~( (1 << 1) | (1 << 0) ); // clear bits 1, 0
  data |= mode; // overwrite bits 1, 0
  writeRegister(BME280_CTRL_MEAS_REG, data);
}

/**@fn writetStandby
   @brief sets sensor standby time (writing the corresponding value in the register)
   @param standby time
   @return none
*/
void BME280::writetStandby(sensortStandby tStandby) {
  uint8_t data = readRegister(BME280_CONFIG_REG);
  data &= ~( (1 << 7) | (1 << 6) | (1 << 5) ); // clear bits 7, 6, 5
  data |= tStandby << 5; // align with bits 7, 6, 5 and overwrite
  writeRegister(BME280_CONFIG_REG, data);
}

/**@fn writeFilter
   @brief sets sensor IIR filter (writing the corresponding value in the register)
   @param filter
   @return none
*/
void BME280::writeFilter(sensorFilter filter) {
  uint8_t data = readRegister(BME280_CONFIG_REG);
  data &= ~( (1 << 4) | (1 << 3) | (1 << 2) ); // clear bits 4, 3, 2
  data |= filter << 2; // align with bits 4, 3, 2 and overwrite
  writeRegister(BME280_CONFIG_REG, data);
}

/**@fn writePressureOS
   @brief sets pressure over sampling (writing the corresponding value in the register)
   @param pressure over sampling
   @return none
*/
void BME280::writePressureOS(sensorSampling overSample) {
  uint8_t data = readRegister(BME280_CTRL_MEAS_REG);
  data &= ~( (1 << 4) | (1 << 3) | (1 << 2) ); // clear bits 4, 3, 2
  data |= overSample << 2; // align with bits 4, 3, 2 and overwrite
  writeRegister(BME280_CTRL_MEAS_REG, data);
}

/**@fn writeTemperatureOS
   @brief sets temperature over sampling (writing the corresponding value in the register)
   @param temperature over sampling
   @return none
*/
void BME280::writeTemperatureOS(sensorSampling overSample) {
  uint8_t data = readRegister(BME280_CTRL_MEAS_REG);
  data &= ~( (1 << 7) | (1 << 6) | (1 << 5) ); // clear bits 7, 6, 5
  data |= overSample << 5; // align with bits 7, 6, 5 and overwrite
  writeRegister(BME280_CTRL_MEAS_REG, data);
}

/**@fn writeHumidityOS
   @brief sets humidity over sampling (writing the corresponding value in the register)
   @param humidity over sampling
   @return none
*/
void BME280::writeHumidityOS(sensorSampling overSample) {
  uint8_t data = readRegister(BME280_CTRL_HUMIDITY_REG);
  data &= ~( (1 << 2) | (1 << 1) | (1 << 0) ); // clear bits 2, 1, 0
  data |= overSample; // align with bits 2, 1, 0 and overwrite
  writeRegister(BME280_CTRL_HUMIDITY_REG, data);
}

/**@fn writeRegister
   @brief writes in the register at the given address the given value
   @param register's address, value to be written
   @return none
*/
void BME280::writeRegister(uint8_t address, uint8_t value) {
  Wire.beginTransmission(settings.I2CAddress);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission();
}

/**@fn readRegister
   @brief reads the value of the register at the given address
   @param register's address
   @return register's value
*/
uint8_t BME280::readRegister(uint8_t address) {
  Wire.beginTransmission(settings.I2CAddress);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(settings.I2CAddress, 1);
  uint8_t value = Wire.read();
  return value;
}

/**@fn readRegisters
   @brief reads the value of s given number of registers from the given address
   @param output array, first address, number of registers
   @return none
*/
void BME280::readRegisters(uint8_t* output, uint8_t address, uint8_t length) {
  Wire.beginTransmission(settings.I2CAddress);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(settings.I2CAddress, length);
  uint8_t i = 0;
  while ((Wire.available()) && (i < length)) {
    uint8_t value = Wire.read();
    *output = value;
    output++;
    i++;
  }
}
