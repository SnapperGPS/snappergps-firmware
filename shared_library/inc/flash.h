/****************************************************************************
 * flash.h
 * SnapperGPS
 * November 2020
 *****************************************************************************/

#ifndef __FLASH_H
#define __FLASH_H

#define FLASH_USART                 USART0

#define FLASH_PAGE_LENGTH           2048

#define FLASH_PAGES_PER_BLOCK       64

#define FLASH_NUM_BLOCKS            512

#define FLASH_NUM_PAGES             (FLASH_NUM_BLOCKS * FLASH_PAGES_PER_BLOCK)

typedef enum {FLASH_SUCCESS, FLASH_ADDRESS_INVALID} FlashResult_t;

void Flash_powerOn(void);

void Flash_powerOff(void);

void Flash_enableInterface(void);

void Flash_disableInterface(void);

void Flash_reset(void);

void Flash_erase(void);

void Flash_blockErase(uint32_t address);

bool Flash_isBusy(void);

void Flash_readID(uint32_t *id);

void Flash_readStatus(uint8_t *status);

void Flash_enterDeepPowerDownMode(void);

void Flash_leaveDeepPowerDownMode(void);

void Flash_readProtectionRegister(uint8_t *status);

void Flash_removeProtection();

void Flash_readConfigurationRegister(uint8_t *status);

void Flash_writeConfigurationRegister(uint8_t *status);

FlashResult_t Flash_readBytes(uint32_t address, uint8_t *dest, uint32_t length);

FlashResult_t Flash_readPage(uint32_t address, uint8_t *dest);

FlashResult_t Flash_writePage(uint32_t address, uint8_t *src);

FlashResult_t Flash_enableContinuousRead(uint32_t address);

uint8_t Flash_readContinuousByte(void);

void Flash_disableContinuousRead(void);

#endif /* __FLASH_H */