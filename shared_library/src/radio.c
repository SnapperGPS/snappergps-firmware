/****************************************************************************
 * radio.c
 * Snapper GPS
 * November 2020
 *****************************************************************************/

#include "em_cmu.h"
#include "em_core.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_device.h"

#include "radio.h"
#include "pinouts.h"

/* Data size constant */

#define BITS_IN_BYTE           8

/* Sample rate constants */

#define RADIO_SAMPLE_RATE       4092000

/* Radio USART settings */

#define RADIO_USART             USART1
#define RADIO_USART_CLK         cmuClock_USART1

/* Public functions */

void Radio_powerOn() {

    GPIO_PinModeSet(RADIO_VDD_EN_PORT, RADIO_VDD_EN_PIN, gpioModePushPull, 0);

}

void Radio_powerOff() {

    GPIO_PinModeSet(RADIO_VDD_EN_PORT, RADIO_VDD_EN_PIN, gpioModeDisabled, 0);

}

void Radio_enableHFXOInput() {

    CMU->CTRL |= CMU_CTRL_HFXOMODE_DIGEXTCLK;

    CMU->OSCENCMD |= CMU_OSCENCMD_HFXOEN;

}

void Radio_disableHFXOInput() {

    CMU->OSCENCMD |= CMU_OSCENCMD_HFXODIS;

}

void Radio_sampleByte(uint8_t *byte) {

    GPIO_PinModeSet(RADIO_SIGN_PORT, RADIO_SIGN_PIN, gpioModeInputPull, 0);

    uint32_t value = 0;
    
    for (uint32_t i = 0; i < BITS_IN_BYTE; i += 1) {

        value += GPIO_PinInGet(RADIO_SIGN_PORT, RADIO_SIGN_PIN) << (BITS_IN_BYTE - 1 - i);
    }

    *byte = (uint8_t)value;

    GPIO_PinModeSet(RADIO_SIGN_PORT, RADIO_SIGN_PIN, gpioModeDisabled, 0);

}

void Radio_captureSnapshot(uint8_t *dest, uint32_t length) {

    // Emable USART clock

    CMU_ClockEnable(RADIO_USART_CLK, true);

    // Enable synchronous mode

    USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;
    
    init.baudrate = RADIO_SAMPLE_RATE;

    init.autoTx = true;

    USART_InitSync(RADIO_USART, &init);

    // Initialise pins and route

    GPIO_PinModeSet(RADIO_SIGN_PORT, RADIO_SIGN_PIN, gpioModeInput, 0);

    RADIO_USART->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_LOCATION_LOC0;

    // Start transmission clock running

    USART_Enable(RADIO_USART, usartEnable);

    RADIO_USART->CMD |= USART_CMD_CLEARRX | USART_CMD_CLEARTX;

    RADIO_USART->TXDATA = (uint32_t)0xFF;

    // Disable interrupts

    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();

    // Main receive loop

    uint8_t *addr = dest;

    while (addr < dest + length) {

        while (!(RADIO_USART->STATUS & USART_STATUS_RXDATAV)) { }

        *addr++ = RADIO_USART->RXDATA;

    }

    // Re-enable interrupts

    CORE_EXIT_CRITICAL();

    // Restore default state

    GPIO_PinModeSet(RADIO_SIGN_PORT, RADIO_SIGN_PIN, gpioModeDisabled, 0);

    USART_Reset(RADIO_USART);

    CMU_ClockEnable(RADIO_USART_CLK, false);

}