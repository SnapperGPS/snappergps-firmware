/****************************************************************************
 * radio.h
 * SnapperGPS
 * November 2020
 *****************************************************************************/

#ifndef __RADIO_H
#define __RADIO_H

void Radio_powerOn(void);

void Radio_powerOff(void);

void Radio_enableHFXOInput(void);

void Radio_disableHFXOInput(void);

void Radio_sampleByte(uint8_t *byte);

void Radio_captureSnapshot(uint8_t *dest, uint32_t length);

#endif /* __RADIO_H */