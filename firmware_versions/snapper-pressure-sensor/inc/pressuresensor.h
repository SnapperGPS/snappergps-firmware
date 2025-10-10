/****************************************************************************
 * pressuresensor.h
 * SnapperGPS
 * Jonas Beuchert
 * June 2023
 * Functions to use an MS5837-02BA a pressure and temperature sensor via I2C
 *****************************************************************************/

#ifndef __PRESSURESENSOR_H
#define __PRESSURESENSOR_H


// Enable I2C for MS5837-02BA
void PressureSensor_enableInterface();

// Disable I2C for MS5837-02BA
void PressureSensor_disableInterface();

// Reset the MS5837-02BA sensor
void PressureSensor_reset();

// Get raw pressure value from MS5837-02BA
uint32_t PressureSensor_getRawPressure();

// Get raw temperature value from MS5837-02BA
uint32_t PressureSensor_getRawTemperature();

int32_t PressureSensor_calculatePressure(uint32_t rawPressure, uint32_t rawTemperature);

#endif /* __PRESSURESENSOR_H */
