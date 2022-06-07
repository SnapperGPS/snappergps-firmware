/****************************************************************************
 * timer.c
 * Snapper GPS
 * November 2020
 *****************************************************************************/

#include "em_cmu.h"
#include "em_emu.h"
#include "em_timer.h"

#include "timer.h"

/* Timer constants */

#define MILLISECONDS_PER_SECOND    1000

#define MICROSECONDS_PER_SECOND    1000000

/* Timer constraints */

#define MAX_MILLISECONDS           2000

#define MAX_MICROSECONDS           2000

/* Useful macros */

#define MIN(a, b)                   ((a) < (b) ? (a) : (b))

/* Timer variable */

static volatile bool timerRunning;

/* Interrupt handler */

void TIMER1_IRQHandler(void) {

    // Get interrupt cause

    uint32_t interruptMask = TIMER_IntGet(TIMER1);

    // Handle interrupt

    if (interruptMask & TIMER_IFC_OF) timerRunning = false;

    // Clear the RTC interrupt flag

    TIMER_IntClear(TIMER1, interruptMask);

}

/* Private function */

static void wait(TIMER_Prescale_TypeDef prescale, uint32_t clockTicksToWait) {

    // Configure timer

    TIMER_Init_TypeDef delayInit = TIMER_INIT_DEFAULT;

    delayInit.prescale = prescale;

    delayInit.enable = false;

    TIMER_Init(TIMER1, &delayInit);

    // Configure interrupt

    TIMER_TopSet(TIMER1, clockTicksToWait);

    TIMER_IntEnable(TIMER1, TIMER_IFS_OF);

    NVIC_ClearPendingIRQ(TIMER1_IRQn);

    NVIC_EnableIRQ(TIMER1_IRQn);

    // Start timer running

    timerRunning = true;

    TIMER_Enable(TIMER1, true);

    while (timerRunning) {

        EMU_EnterEM1();

    }

    // Stop the timer

    TIMER_Reset(TIMER1);

    NVIC_DisableIRQ(TIMER1_IRQn);

}

/* Public function*/

void Timer_enable() {

    CMU_ClockEnable(cmuClock_TIMER1, true);

}

void Timer_disable() {

    CMU_ClockEnable(cmuClock_TIMER1, false);

}

void Timer_delayMilliseconds(uint32_t milliseconds) {

    milliseconds = MIN(milliseconds, MAX_MILLISECONDS);

    uint32_t timerClockFrequency = CMU_ClockFreqGet(cmuClock_TIMER1);

    uint32_t clockTicksToWait = ((timerClockFrequency >> timerPrescale1024) * milliseconds) / MILLISECONDS_PER_SECOND;

    wait(timerPrescale1024, clockTicksToWait);

}

void Timer_delayMicroseconds(uint32_t microseconds) {

    microseconds = MIN(microseconds, MAX_MICROSECONDS);
    
    uint32_t timerClockFrequency = CMU_ClockFreqGet(cmuClock_TIMER1);

    uint32_t clockTicksToWait = (timerClockFrequency / MICROSECONDS_PER_SECOND) * microseconds;

    wait(timerPrescale1, clockTicksToWait);

}