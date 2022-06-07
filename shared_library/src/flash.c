/****************************************************************************
 * flash.c
 * SnapperGPS
 * November 2020
 *****************************************************************************/

#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_device.h"

#include "flash.h"
#include "pinouts.h"

/* SPI USART settings */

#define FLASH_USART_CLK         cmuClock_USART0

/* Flash instructions (W25N512GV) */

#define FLASH_CMD_PP            0x02        // Load program (1 to 2112 bytes)
#define FLASH_CMD_EXEC          0x10        // Program execute
#define FLASH_CMD_PAGE_READ     0x13        // Page data read
#define FLASH_CMD_READ          0x03        // Read data (after read paga data)
#define FLASH_CMD_RDSR          0x05        // Read status register
#define FLASH_CMD_WREN          0x06        // Write enable
#define FLASH_CMD_CE            0x60        // Chip erase (may take several seconds)
#define FLASH_CMD_BE            0xD8        // Block erase
#define FLASH_CMD_RSTEN         0x66        // Enable reset
#define FLASH_CMD_RST           0x99        // Reset device
#define FLASH_CMD_RDID          0x9F        // Read JEDEC ID
#define FLASH_CMD_DP            0xB9        // Deep power down
#define FLASH_CMD_RDP           0xAB        // Release power-down
#define FLASH_CMD_WRSR          0x01        // Write status register
#define FLASH_CMD_BAD_BLK_MGMT  0xA1        // Bad block management
#define FLASH_CMD_READ_BBM_LUT  0xA5        // Read bad block management look-up table
#define FLASH_ADDR_PROTECTION   0xA0        // Status register 1 address (protection register)
#define FLASH_ADDR_CONFIG       0xB0        // Status register 2 address (configuration register)
#define FLASH_ADDR_STATUS       0xC0        // Status register 3 address
#define FLASH_CMD_DUMMY         0x00        // Dummy byte

#define FLASH_MASK_BUF          0b00001000  // Mask for 'buffer read' in configuration register

/* Macros */

#define CS_LOW                   GPIO_PinModeSet(FLASH_CS_PORT, FLASH_CS_PIN, gpioModePushPull, 0)

#define CS_HIGH                  GPIO_PinModeSet(FLASH_CS_PORT, FLASH_CS_PIN, gpioModeDisabled, 0)

/* Private functions */

static inline uint8_t getByte() {

   return USART_SpiTransfer(FLASH_USART, 0xFF);

}

static inline void sendByte(uint8_t byte) {

    USART_SpiTransfer(FLASH_USART, byte);

}

static inline void sendAddress(uint32_t address) {

    USART_SpiTransfer(FLASH_USART, address >> 8);
    USART_SpiTransfer(FLASH_USART, address);

}

/* Public functions */

void Flash_powerOn(void) {

    GPIO_PinModeSet(FLASH_CS_PORT, FLASH_CS_PIN, gpioModeDisabled, 0);

    GPIO_PinModeSet(FLASH_VDD_EN_PORT, FLASH_VDD_EN_PIN, gpioModePushPull, 0);

}

void Flash_powerOff(void) {

    GPIO_PinModeSet(FLASH_CS_PORT, FLASH_CS_PIN, gpioModeDisabled, 0);

    GPIO_PinModeSet(FLASH_VDD_EN_PORT, FLASH_VDD_EN_PIN, gpioModeDisabled, 0);

}

void Flash_enableInterface() {

    CMU_ClockEnable(FLASH_USART_CLK, true);

    USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;

    init.msbf = true;

    init.baudrate = 7000000;

    USART_InitSync(FLASH_USART, &init);

    GPIO_PinModeSet(FLASH_MOSI_PORT, FLASH_MOSI_PIN, gpioModePushPull, 1);

    GPIO_PinModeSet(FLASH_MISO_PORT, FLASH_MISO_PIN, gpioModeInput, 0);

    GPIO_PinModeSet(FLASH_SCLK_PORT, FLASH_SCLK_PIN, gpioModePushPull, 1);

    FLASH_USART->ROUTE = USART_ROUTE_CLKPEN | USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_LOCATION_LOC0;

    USART_Enable(FLASH_USART, usartEnable);

}

void Flash_disableInterface() {

    GPIO_PinModeSet(FLASH_MOSI_PORT, FLASH_MOSI_PIN, gpioModeDisabled, 0);

    GPIO_PinModeSet(FLASH_MISO_PORT, FLASH_MISO_PIN, gpioModeDisabled, 0);

    GPIO_PinModeSet(FLASH_SCLK_PORT, FLASH_SCLK_PIN, gpioModeDisabled, 0);

    USART_Reset(FLASH_USART);

    CMU_ClockEnable(FLASH_USART_CLK, false);

}

void Flash_readID(uint32_t *id) {

    uint8_t data[3];

    CS_LOW;

    sendByte(FLASH_CMD_RDID);

    sendByte(FLASH_CMD_DUMMY);

    data[0] = getByte();
    data[1] = getByte();
    data[2] = getByte();

    CS_HIGH;

    *id = (data[0] << 16) | (data[1] << 8) | data[2];

}

void Flash_reset() {

    CS_LOW;

    sendByte(FLASH_CMD_RSTEN);

    CS_HIGH;

    CS_LOW;

    sendByte(FLASH_CMD_RST);

    CS_HIGH;

}

void Flash_erase() {

    CS_LOW;

    sendByte(FLASH_CMD_WREN);

    CS_HIGH;

    CS_LOW;

    sendByte(FLASH_CMD_CE);

    CS_HIGH;

}

void Flash_blockErase(uint32_t address) {

    CS_LOW;

    sendByte(FLASH_CMD_WREN);

    CS_HIGH;

    CS_LOW;

    sendByte(FLASH_CMD_BE);

    sendByte(FLASH_CMD_DUMMY);

    sendAddress(address);

    CS_HIGH;

}

void Flash_enterDeepPowerDownMode() {

    CS_LOW;

    sendByte(FLASH_CMD_DP);

    CS_HIGH;

}

void Flash_leaveDeepPowerDownMode() {

    CS_LOW;

    sendByte(FLASH_CMD_RDP);

    CS_HIGH;

}

bool Flash_isBusy() {

    uint8_t status;

    Flash_readStatus(&status);

    return (status & 0x01) == 0x01;

}

void Flash_readStatus(uint8_t *status) {

    CS_LOW;

    sendByte(FLASH_CMD_RDSR);

    sendByte(FLASH_ADDR_STATUS);

    *status = getByte();

    CS_HIGH;

}

void Flash_readProtectionRegister(uint8_t *status) {

    CS_LOW;

    sendByte(FLASH_CMD_RDSR);

    sendByte(FLASH_ADDR_PROTECTION);

    *status = getByte();

    CS_HIGH;

}

void Flash_removeProtection() {

    CS_LOW;

    sendByte(FLASH_CMD_WRSR);

    sendByte(FLASH_ADDR_PROTECTION);

    sendByte(0x00);

    CS_HIGH;

}

void Flash_readConfigurationRegister(uint8_t *status) {

    CS_LOW;

    sendByte(FLASH_CMD_RDSR);

    sendByte(FLASH_ADDR_CONFIG);

    *status = getByte();

    CS_HIGH;

}

void Flash_writeConfigurationRegister(uint8_t *status) {

    CS_LOW;

    sendByte(FLASH_CMD_WRSR);

    sendByte(FLASH_ADDR_CONFIG);

    sendByte(*status);

    CS_HIGH;

}

FlashResult_t Flash_readBytes(uint32_t address, uint8_t *dest, uint32_t length) {

    if (address >= FLASH_NUM_PAGES) return FLASH_ADDRESS_INVALID;

    if (length > FLASH_PAGE_LENGTH) length = FLASH_PAGE_LENGTH;

    //  Transfer data of specified memory page into data buffer.

    CS_LOW;

    sendByte(FLASH_CMD_PAGE_READ);

    sendByte(FLASH_CMD_DUMMY);

    sendAddress(address);

    CS_HIGH;

    while (Flash_isBusy());

    CS_LOW;

    sendByte(FLASH_CMD_READ);

    sendByte(FLASH_CMD_DUMMY);
    sendByte(FLASH_CMD_DUMMY);
    sendByte(FLASH_CMD_DUMMY);

    for (uint32_t i = 0; i < length; i++) dest[i] = getByte();

    CS_HIGH;

    return FLASH_SUCCESS;

}

FlashResult_t Flash_readPage(uint32_t address, uint8_t *dest) {

    return Flash_readBytes(address, dest, FLASH_PAGE_LENGTH);

}

FlashResult_t Flash_writePage(uint32_t address, uint8_t *src) {

    if (address >= FLASH_NUM_PAGES) return FLASH_ADDRESS_INVALID;

    CS_LOW;

    sendByte(FLASH_CMD_WREN);

    CS_HIGH;

    while (Flash_isBusy());

    CS_LOW;

    sendByte(FLASH_CMD_PP);

    sendByte(FLASH_CMD_DUMMY);
    sendByte(FLASH_CMD_DUMMY);

    for (uint32_t i=0; i < FLASH_PAGE_LENGTH; i++) sendByte(src[i]);

    CS_HIGH;

    while (Flash_isBusy());

    // Program data from data buffer into memory

    CS_LOW;

    sendByte(FLASH_CMD_EXEC);

    sendByte(FLASH_CMD_DUMMY);

    sendAddress(address);

    CS_HIGH;

    return FLASH_SUCCESS;

}

FlashResult_t Flash_enableContinuousRead(uint32_t address) {

    if (address >= FLASH_NUM_PAGES) return FLASH_ADDRESS_INVALID;

    while (Flash_isBusy());

    // Enable continous read mode (BUF=0)

    uint8_t config;
    Flash_readConfigurationRegister(&config);
    config = config & ~FLASH_MASK_BUF;
    while (Flash_isBusy());
    Flash_writeConfigurationRegister(&config);
    while (Flash_isBusy());

    // Transfer data of specified memory page into data buffer.

    CS_LOW;

    sendByte(FLASH_CMD_PAGE_READ);

    sendByte(FLASH_CMD_DUMMY);

    sendAddress(address);

    CS_HIGH;

    while (Flash_isBusy());

    CS_LOW;

    sendByte(FLASH_CMD_READ);

    sendByte(FLASH_CMD_DUMMY);
    sendByte(FLASH_CMD_DUMMY);
    sendByte(FLASH_CMD_DUMMY);

    return FLASH_SUCCESS;

}

uint8_t Flash_readContinuousByte() {

    return getByte();

}

void Flash_disableContinuousRead() {

    CS_HIGH;

}
