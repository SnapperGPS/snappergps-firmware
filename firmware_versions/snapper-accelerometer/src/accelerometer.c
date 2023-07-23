/****************************************************************************
 * accelerometer.c
 * SnapperGPS
 * Jonas Beuchert
 * June 2023
 * Functions to use an LIS3DH-based 3-axis MEMS accelerometer via I2C
 *****************************************************************************/

#include "em_cmu.h"
#include "em_emu.h"
#include "em_i2c.h"
#include "em_gpio.h"

// #include "arm_math.h"

#include "timer.h"
#include "pinouts.h"

#include "accelerometer.h"

// Ports and pins for I2C
#define I2C_SDA_PORT            gpioPortA
#define I2C_SDA_PIN             0
#define I2C_SCL_PORT            gpioPortA
#define I2C_SCL_PIN             1
#define I2C_LOC                 I2C_ROUTE_LOCATION_LOC0

// The default I2C Address to use for the LIS3DH
// I2C Address selection pin: when SA0=L, I2C address = 0x18, when ADDR=H, I2C
// address = 0x19.
#define LIS3DH_I2C_ADDRESS          0b0011001

// Registers
#define LIS3DH_AUX          0x07
#define LIS3DH_CTRL_REG0    0x1E
#define LIS3DH_CTRL_REG1    0x20
#define LIS3DH_CTRL_REG2    0x21
#define LIS3DH_CTRL_REG3    0x22
#define LIS3DH_CTRL_REG4    0x23
#define LIS3DH_CTRL_REG5    0x24
#define LIS3DH_CTRL_REG6    0x25
#define LIS3DH_OUT_X_L      0x28
#define LIS3DH_OUT_X_H      0x29
#define LIS3DH_OUT_Y_L      0x2A
#define LIS3DH_OUT_Y_H      0x2B
#define LIS3DH_OUT_Z_L      0x2C
#define LIS3DH_OUT_Z_H      0x2D
#define LIS3DH_FIFO_CTRL    0x2E
#define LIS3DH_WHO_AM_I     0x0F

// Content of the WHO_AM_I register
#define LIS3DH_ID                   0b00110011

// Constants
#define TIMEOUT             10000

// I2C variables

static volatile I2C_TransferReturn_TypeDef status;

static I2C_TransferSeq_TypeDef seq = {.addr = (LIS3DH_I2C_ADDRESS << 1)};

// Private functions

// Interrupt handler
void I2C0_IRQHandler(void) {

    status = I2C_Transfer(I2C0);

}

static void write(uint8_t *buffer_, uint16_t size_) {
    // Adapted from AN0011
    seq.flags = I2C_FLAG_WRITE;
    seq.buf[0].data = buffer_;
    seq.buf[0].len = size_;
    // Do a polled transfer
    status = I2C_TransferInit(I2C0, &seq);
    while (status == i2cTransferInProgress) {
        // Enter EM1 while waiting for I2C interrupt
        EMU_EnterEM1();
        // Could do a timeout function here
    }
}

static void read(uint8_t *message, uint8_t *data, int32_t messageLength, int32_t dataLength) {

    seq.flags = I2C_FLAG_WRITE_READ;

    seq.buf[0].data = message;
    seq.buf[0].len = messageLength;

    seq.buf[1].data = data;
    seq.buf[1].len = dataLength;

    status = I2C_TransferInit(I2C0, &seq);

    uint32_t counter = 0;

    while (status == i2cTransferInProgress && counter < TIMEOUT) {

        // EMU_EnterEM1();

        ++counter;

    }

}

static void write8bit(uint8_t reg, uint8_t value) {
    uint8_t i2cBufferOut[2];
    i2cBufferOut[0] = reg;
    i2cBufferOut[1] = value;
    write(i2cBufferOut, 2);
}

static void read8bit(uint8_t start_reg, uint8_t *value) {
    uint8_t i2cBufferOut[1];
    i2cBufferOut[0] = start_reg;
    uint8_t i2cBufferIn[1];
    read(i2cBufferOut, i2cBufferIn, 1, 1);
    *value = i2cBufferIn[0];
}

void selectFifoMode(uint8_t mode) {
    uint8_t config = 0;
    // Read the current value of the control register
    read8bit(LIS3DH_FIFO_CTRL, &config);
    // Clear the fifo mode bits
    config &= 0b00111111;
    // Set the fifo mode bits
    config |= mode << 6;
    write8bit(LIS3DH_FIFO_CTRL, config);
}


// Public functions

uint8_t Accelerometer_whoAmI() {
    uint8_t whoAmI = 0;
    read8bit(LIS3DH_WHO_AM_I, &whoAmI);
    return whoAmI;
}

bool Accelerometer_isNewDataAvailable() {
    uint8_t status = 0;
    read8bit(LIS3DH_AUX, &status);
    return (status & 0b00001000) >> 3;
}

void Accelerometer_readXYZ(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t i2cBufferOut[1];
    i2cBufferOut[0] = LIS3DH_OUT_X_L | 0x80;
    uint8_t i2cBufferIn[6];
    read(i2cBufferOut, i2cBufferIn, 1, 6);
    *x = 0;
    *x |= (uint16_t) i2cBufferIn[0] << 8;
    *x |= i2cBufferIn[1];
    *y = 0;
    *y |= (uint16_t) i2cBufferIn[2] << 8;
    *y |= i2cBufferIn[3];
    *z = 0;
    *z |= (uint16_t) i2cBufferIn[4] << 8;
    *z |= i2cBufferIn[5];
}

void Accelerometer_selectDataRate(uint16_t frequency) {
    uint8_t config = 0;
    // Read the current value of the control register
    read8bit(LIS3DH_CTRL_REG1, &config);
    // Clear the data rate bits
    config &= 0b00001111;
    // Set the data rate bits
    switch (frequency)
    {
    case 0:
        config |= 0b00000000;
        break;
    case 1:
        config |= 0b00010000;
        break;
    case 10:
        config |= 0b00100000;
        break;
    case 25:
        config |= 0b00110000;
        break;
    case 50:
        config |= 0b01000000;
        break;
    case 100:
        config |= 0b01010000;
        break;
    case 200:
        config |= 0b01100000;
        break;
    case 400:
        config |= 0b01110000;
        break;
    case 1600:
        config |= 0b10000000;
        break;
    case 1344:
        config |= 0b10010000;
        break;
    case 5376:
        config |= 0b10100000;
        break;
    default:
        break;
    }
    write8bit(LIS3DH_CTRL_REG1, config);
}

void Accelerometer_enableLowPowerMode() {
    uint8_t config = 0;
    // Read the current value of the control register
    read8bit(LIS3DH_CTRL_REG1, &config);
    // Set the low power mode bit
    config |= 0b00001000;
    write8bit(LIS3DH_CTRL_REG1, config);
}

void Accelerometer_disableLowPowerMode() {
    uint8_t config = 0;
    // Read the current value of the control register
    read8bit(LIS3DH_CTRL_REG1, &config);
    // Clear the low power mode bit
    config &= 0b11110111;
    write8bit(LIS3DH_CTRL_REG1, config);
}

void Accelerometer_selectFifoModeBypass() {
    selectFifoMode(0);
}

void Accelerometer_selectFifoModeFifo() {
    selectFifoMode(1);
}

void Accelerometer_selectFifoModeStream() {
    selectFifoMode(2);
}

void Accelerometer_selectFifoModeStreamToFifo() {
    selectFifoMode(3);
}

void Accelerometer_enableHighResolutionOutputMode() {
    uint8_t config = 0;
    // Read the current value of the control register
    read8bit(LIS3DH_CTRL_REG4, &config);
    // Set the high resolution output mode bit
    config |= 0b00001000;
    write8bit(LIS3DH_CTRL_REG4, config);
}

void Accelerometer_disableHighResolutionOutputMode() {
    uint8_t config = 0;
    // Read the current value of the control register
    read8bit(LIS3DH_CTRL_REG4, &config);
    // Clear the high resolution output mode bit
    config &= 0b11110111;
    write8bit(LIS3DH_CTRL_REG4, config);
}

void Accelerometer_enableBlockDataUpdate() {
    uint8_t config = 0;
    // Read the current value of the control register
    read8bit(LIS3DH_CTRL_REG4, &config);
    // Set the block data update bit
    config |= 0b10000000;
    write8bit(LIS3DH_CTRL_REG4, config);
}

void Accelerometer_disableBlockDataUpdate() {
    uint8_t config = 0;
    // Read the current value of the control register
    read8bit(LIS3DH_CTRL_REG4, &config);
    // Clear the block data update bit
    config &= 0b01111111;
    write8bit(LIS3DH_CTRL_REG4, config);
}

void Accelerometer_selectScale(uint8_t g) {
    uint8_t config = 0;
    // Read the current value of the control register
    read8bit(LIS3DH_CTRL_REG4, &config);
    // Clear the scale bits
    config &= 0b11001111;
    // Set the scale bits
    switch (g)
    {
    case 2:
        config |= 0b00000000;
        break;
    case 4:
        config |= 0b00010000;
        break;
    case 8:
        config |= 0b00100000;
        break;
    case 16:
        config |= 0b00110000;
        break;
    default:
        break;
    }
    write8bit(LIS3DH_CTRL_REG4, config);
}

void Accelerometer_readCtrlReg(uint8_t *ctrlReg) {
    read8bit(LIS3DH_CTRL_REG0, &ctrlReg[0]);
    read8bit(LIS3DH_CTRL_REG1, &ctrlReg[1]);
    read8bit(LIS3DH_CTRL_REG2, &ctrlReg[2]);
    read8bit(LIS3DH_CTRL_REG3, &ctrlReg[3]);
    read8bit(LIS3DH_CTRL_REG4, &ctrlReg[4]);
    read8bit(LIS3DH_CTRL_REG5, &ctrlReg[5]);
    read8bit(LIS3DH_CTRL_REG6, &ctrlReg[6]);
}

void Accelerometer_enableInterface() {

    CMU_ClockEnable(cmuClock_I2C0, true);

    GPIO_PinModeSet(I2C_SDA_PORT, I2C_SDA_PIN, gpioModeWiredAnd, 1);

    GPIO_PinModeSet(I2C_SCL_PORT, I2C_SCL_PIN, gpioModeWiredAnd, 1);

    for (int i = 0; i < 9; i++) {
        GPIO_PinModeSet(I2C_SCL_PORT, I2C_SCL_PIN, gpioModeWiredAnd, 0);
        GPIO_PinModeSet(I2C_SCL_PORT, I2C_SCL_PIN, gpioModeWiredAnd, 1);
    }

    I2C0->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN | I2C_LOC;

    I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

    I2C_Init(I2C0, &i2cInit);

    NVIC_ClearPendingIRQ(I2C0_IRQn);

    NVIC_EnableIRQ(I2C0_IRQn);

}

void Accelerometer_disableInterface() {

    GPIO_PinModeSet(I2C_SDA_PORT, I2C_SDA_PIN, gpioModeDisabled, 0);

    GPIO_PinModeSet(I2C_SCL_PORT, I2C_SCL_PIN, gpioModeDisabled, 0);

    I2C_Reset(I2C0);

    CMU_ClockEnable(cmuClock_I2C0, false);

    NVIC_DisableIRQ(I2C0_IRQn);

}

bool Accelerometer_isAvailable() {

    uint8_t whoAmI = Accelerometer_whoAmI();

    return whoAmI == LIS3DH_ID;

}
