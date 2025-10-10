/****************************************************************************
 * pressuresensor.c
 * SnapperGPS
 * Jonas Beuchert
 * June 2023
 * Functions to use an MS5837-02BA a pressure and temperature sensor via I2C
 *****************************************************************************/

#include "em_cmu.h"
#include "em_emu.h"
#include "em_i2c.h"
#include "em_gpio.h"

// #include "arm_math.h"

#include "timer.h"
#include "pinouts.h"

#include "pressuresensor.h"

// Ports and pins for I2C
#define I2C_SDA_PORT            gpioPortA
#define I2C_SDA_PIN             0
#define I2C_SCL_PORT            gpioPortA
#define I2C_SCL_PIN             1
#define I2C_LOC                 I2C_ROUTE_LOCATION_LOC0

// The default I2C Address to use for the MS5837-02BA
#define MS5837_02BA_I2C_ADDRESS          0b1110110

// Commands
#define MS5837_02BA_RESET                 0x1E // The Reset sequence shall be sent once after power-on
#define MS5837_02BA_CONVERT_D1_OSR_256    0x40 // Initiate uncompensated pressure (D1) conversion
#define MS5837_02BA_CONVERT_D1_OSR_512    0x42
#define MS5837_02BA_CONVERT_D1_OSR_1024   0x44
#define MS5837_02BA_CONVERT_D1_OSR_2048   0x46
#define MS5837_02BA_CONVERT_D1_OSR_4096   0x48
#define MS5837_02BA_CONVERT_D1_OSR_8192   0x4A
#define MS5837_02BA_CONVERT_D2_OSR_256    0x50 // Initiate uncompensated temperature (D2) conversion
#define MS5837_02BA_CONVERT_D2_OSR_512    0x52
#define MS5837_02BA_CONVERT_D2_OSR_1024   0x54
#define MS5837_02BA_CONVERT_D2_OSR_2048   0x56
#define MS5837_02BA_CONVERT_D2_OSR_4096   0x58
#define MS5837_02BA_CONVERT_D2_OSR_8192   0x5A
#define MS5837_02BA_ADC_READ              0x00
#define MS5837_02BA_PROM_READ_BASE        0xA0 // PROM Read commands range from 0xA0 to 0xAC, 16 bit each
/* The read command for PROM shall be executed once after reset by the user to read
the content of the calibration  PROM and to calculate the calibration coefficients. */

#define MS5837_02BA_MAX_CONVERSION_TIME_OSR_256    1
#define MS5837_02BA_MAX_CONVERSION_TIME_OSR_512    2
#define MS5837_02BA_MAX_CONVERSION_TIME_OSR_1024   3
#define MS5837_02BA_MAX_CONVERSION_TIME_OSR_2048   5
#define MS5837_02BA_MAX_CONVERSION_TIME_OSR_4096   9
#define MS5837_02BA_MAX_CONVERSION_TIME_OSR_8192   18

// Constants
#define TIMEOUT             10000

// I2C variables

static volatile I2C_TransferReturn_TypeDef status;

static I2C_TransferSeq_TypeDef seq = {.addr = (MS5837_02BA_I2C_ADDRESS << 1)};

// MS5837-02BA configuration

uint16_t osr = 8192; // Oversampling rate, default is 8192

static uint16_t promArray[7] = {0};

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

static void write8bit(uint8_t value) {
    uint8_t i2cBufferOut[1];
    i2cBufferOut[0] = value;
    write(i2cBufferOut, 1);
}

static void read8bit(uint8_t start_reg, uint8_t *value) {
    uint8_t i2cBufferOut[1];
    i2cBufferOut[0] = start_reg;
    uint8_t i2cBufferIn[1];
    read(i2cBufferOut, i2cBufferIn, 1, 1);
    *value = i2cBufferIn[0];
}

static void read16bit(uint8_t start_reg, uint16_t *value) {
    uint8_t i2cBufferOut[1];
    i2cBufferOut[0] = start_reg;
    uint8_t i2cBufferIn[2];
    read(i2cBufferOut, i2cBufferIn, 1, 2);
    *value = 0;
    *value |= (uint16_t)i2cBufferIn[0] << 8;
    *value |= i2cBufferIn[1];
}

static void read24bit(uint8_t start_reg, uint32_t *value) {
    uint8_t i2cBufferOut[1];
    i2cBufferOut[0] = start_reg;
    uint8_t i2cBufferIn[3];
    read(i2cBufferOut, i2cBufferIn, 1, 3);
    *value = 0;
    *value |= (uint32_t)i2cBufferIn[0] << 16;
    *value |= (uint32_t)i2cBufferIn[1] << 8;
    *value |= i2cBufferIn[2];
}

void PressureSensor_enableInterface() {

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

void PressureSensor_disableInterface() {

    GPIO_PinModeSet(I2C_SDA_PORT, I2C_SDA_PIN, gpioModeDisabled, 0);

    GPIO_PinModeSet(I2C_SCL_PORT, I2C_SCL_PIN, gpioModeDisabled, 0);

    I2C_Reset(I2C0);

    CMU_ClockEnable(cmuClock_I2C0, false);

    NVIC_DisableIRQ(I2C0_IRQn);

}

void PressureSensor_reset() {

    // Reset the sensor
    write8bit(MS5837_02BA_RESET);

    // Wait for reset to complete
    Timer_delayMilliseconds(10);

    
    for (uint8_t i = 0; i < 7; ++i) {
        read16bit(MS5837_02BA_PROM_READ_BASE + i * 2, promArray + i);
    }

}

uint32_t getResult(bool pressure) {

    // Start pressure conversion and wait
    switch (osr) {
        case 256:
            if (pressure) write8bit(MS5837_02BA_CONVERT_D1_OSR_256); else write8bit(MS5837_02BA_CONVERT_D2_OSR_256);
            Timer_delayMilliseconds(MS5837_02BA_MAX_CONVERSION_TIME_OSR_256);
            break;
        case 512:
            if (pressure) write8bit(MS5837_02BA_CONVERT_D1_OSR_512); else write8bit(MS5837_02BA_CONVERT_D2_OSR_512);
            Timer_delayMilliseconds(MS5837_02BA_MAX_CONVERSION_TIME_OSR_512);
            break;
        case 1024:
            if (pressure) write8bit(MS5837_02BA_CONVERT_D1_OSR_1024); else write8bit(MS5837_02BA_CONVERT_D2_OSR_1024);
            Timer_delayMilliseconds(MS5837_02BA_MAX_CONVERSION_TIME_OSR_1024);
            break;
        case 2048:
            if (pressure) write8bit(MS5837_02BA_CONVERT_D1_OSR_2048); else write8bit(MS5837_02BA_CONVERT_D2_OSR_2048);
            Timer_delayMilliseconds(MS5837_02BA_MAX_CONVERSION_TIME_OSR_2048);
            break;
        case 4096:
            if (pressure) write8bit(MS5837_02BA_CONVERT_D1_OSR_4096); else write8bit(MS5837_02BA_CONVERT_D2_OSR_4096);
            Timer_delayMilliseconds(MS5837_02BA_MAX_CONVERSION_TIME_OSR_4096);
            break;
        case 8192:
            if (pressure) write8bit(MS5837_02BA_CONVERT_D1_OSR_8192); else write8bit(MS5837_02BA_CONVERT_D2_OSR_8192);
            Timer_delayMilliseconds(MS5837_02BA_MAX_CONVERSION_TIME_OSR_8192);
            break;
        default:
            return 0;
    }

    // Read pressure
    uint32_t result;
    read24bit(MS5837_02BA_ADC_READ, &result);
    return result;

}

uint32_t PressureSensor_getRawPressure() {
    return getResult(true);
}

uint32_t PressureSensor_getRawTemperature() {
    return getResult(false);
}

int32_t PressureSensor_calculatePressure(uint32_t rawPressure, uint32_t rawTemperature) {
    int32_t dT = rawTemperature - (int32_t)promArray[5] * 256;
    // std::cout << "dT = " << dT << "\n";
    int32_t temp = 2000LL + ((int64_t)dT * (int64_t)promArray[6]) / 8388608LL;
    // std::cout << "TEMP = " << temp << "\n";
    int64_t off = (int64_t)promArray[2] * 131072LL + ((int64_t)promArray[4] * (int64_t)dT) / 64LL;
    // std::cout << "OFF = " << off << "\n";
    int64_t sens = (int64_t)promArray[1] * 65536LL + ((int64_t)promArray[3] * (int64_t)dT) / 128LL;
    // std::cout << "SENS = " << sens << "\n";
    int32_t pressure = (((int64_t)rawPressure * sens) / 2097152LL - off) / 32768LL;
    // std::cout << "P = " << pressure << "\n";

    // Second order temperature compensation for MS5837-02BA01
    if (temp < 2000) {
        int64_t Ti = (11LL * dT * dT) / 34359738368LL; 
        // std::cout << "Ti = " << Ti << "\n";
        int64_t offi = (31LL * ((int64_t)temp - 2000LL) * ((int64_t)temp - 2000LL)) / 8LL;
        // std::cout << "OFFi = " << offi << "\n";
        int64_t sensi = (63LL * ((int64_t)temp - 2000LL) * ((int64_t)temp - 2000LL)) / 32LL;
        // std::cout << "SENSi = " << sensi << "\n";
        int64_t off2 = off - offi;
        // std::cout << "OFF2 = " << off2 << "\n";
        int64_t sens2 = sens - sensi;
        // std::cout << "SENS2 = " << sens2 << "\n";
        pressure = (int32_t)((((int64_t)rawPressure * sens2) / 2097152LL - off2) / 32768LL);
    }

    return pressure;
}
