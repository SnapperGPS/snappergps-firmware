/****************************************************************************
 * analogToDigitalConverter.h
 * SnapperGPS
 * November 2020
 *****************************************************************************/

#ifndef __ANALOG_TO_DIGITAL_CONVERTER_H
#define __ANALOG_TO_DIGITAL_CONVERTER_H

#include <stdint.h>

/* Enumeration whether pin shall be pulled low or high */

typedef enum {PULL_LOW = 0, PULL_HIGH = 1} pinPullValue_t;

void AnalogToDigitalConverter_enable(void);

void AnalogToDigitalConverter_disable(void);

void AnalogToDigitalConverter_enableBatteryMeasurement(pinPullValue_t enablePullValue);

void AnalogToDigitalConverter_disableBatteryMeasurement(void);

uint32_t AnalogToDigitalConverter_measureVDD(void);

int32_t AnalogToDigitalConverter_measureTemperature(void);

uint32_t AnalogToDigitalConverter_measureBatteryVoltage(void);

#endif /* __ANALOG_TO_DIGITAL_CONVERTER_H */