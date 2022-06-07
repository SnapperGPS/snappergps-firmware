/****************************************************************************
 * analogToDigitalConverter.c
 * SnapperGPS
 * November 2020
 *****************************************************************************/

#include "em_cmu.h"
#include "em_adc.h"

#include "pinouts.h"
#include "analogToDigitalConverter.h"

/* ADC constants */

#define ADC_FREQ                    1000000

/* Voltage constants */

#define ADC_1V25_REF                125
#define ADC_2V5_REF                 250

#define ADC_RES                     65535

/* Temperature constants */

#define DECIDEGREES_IN_DEGREE       10
#define TEMPERATURE_GRADIENT        63
#define GRADIENT_MULTIPLIER         10

/* Useful macro */

#define ROUNDED_DIV(a, b)       (((a) + (b/2)) / (b))

/* Static functions */

static void initialiseADC() {

    ADC_Init_TypeDef init = ADC_INIT_DEFAULT;

    init.prescale = ADC_PrescaleCalc(ADC_FREQ, 0);

    init.timebase = ADC_TimebaseCalc(0);

    init.ovsRateSel = adcOvsRateSel16;

    ADC_Init(ADC0, &init);

}

static uint32_t makeMeasurement() {

    ADC_Start(ADC0, adcStartSingle);

    while (ADC0->STATUS & ADC_STATUS_SINGLEACT);

    return ADC_DataSingleGet(ADC0);

}

/* Global functions */

void AnalogToDigitalConverter_enable() {

    CMU_ClockEnable(cmuClock_ADC0, true);

}

void AnalogToDigitalConverter_disable() {

    CMU_ClockEnable(cmuClock_ADC0, false);

}

void AnalogToDigitalConverter_enableBatteryMeasurement(pinPullValue_t enablePullValue) {

    GPIO_PinModeSet(VBAT_SENSE_EN_PORT, VBAT_SENSE_EN_PIN, gpioModePushPull, enablePullValue);

}

void AnalogToDigitalConverter_disableBatteryMeasurement() {

    GPIO_PinModeSet(VBAT_SENSE_EN_PORT, VBAT_SENSE_EN_PIN, gpioModeDisabled, 0);

}

uint32_t AnalogToDigitalConverter_measureVDD() {

    // Initialise ADC

    initialiseADC();

    // Initialise ADC for single measurement

    ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

    initSingle.acqTime = adcAcqTime32;

    initSingle.resolution = adcResOVS;

    initSingle.input = adcSingleInputVDDDiv3;

    ADC_InitSingle(ADC0, &initSingle);

    // Calculate voltage

    uint32_t adcSample = makeMeasurement();

    uint32_t voltage = ROUNDED_DIV(3 * adcSample * ADC_1V25_REF, ADC_RES);

    return voltage;

}

int32_t AnalogToDigitalConverter_measureTemperature() {

    // Initialise ADC

    initialiseADC();

    // Initialise ADC for single measurement

    ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

    initSingle.acqTime = adcAcqTime32;

    initSingle.input = adcSingleInpTemp;

    ADC_InitSingle(ADC0, &initSingle);

    // Calculate temperature

    int32_t adcSample = makeMeasurement();

    uint32_t CAL_TEMP_0 = ((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK) >> _DEVINFO_CAL_TEMP_SHIFT);

    if ((CAL_TEMP_0 == 0xFF) || (CAL_TEMP_0 == 0xFFF)) return -10000;

    int32_t ADC0_TEMP_0_READ_1V25 = ((DEVINFO->ADC0CAL2 & _DEVINFO_ADC0CAL2_TEMP1V25_MASK) >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);

    int32_t temperature = DECIDEGREES_IN_DEGREE * CAL_TEMP_0;

    temperature += ROUNDED_DIV(DECIDEGREES_IN_DEGREE * GRADIENT_MULTIPLIER * (ADC0_TEMP_0_READ_1V25 - adcSample), TEMPERATURE_GRADIENT);

    return temperature;

}

uint32_t AnalogToDigitalConverter_measureBatteryVoltage() {

    // Initialise ADC

    initialiseADC();

    // Initialise ADC for single measurement

    ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

    initSingle.acqTime = adcAcqTime32;

    initSingle.reference = adcRef2V5;

    initSingle.resolution = adcResOVS;

    initSingle.input = adcSingleInpCh7;

    ADC_InitSingle(ADC0, &initSingle);

    // Calculate voltage

    uint32_t adcSample = makeMeasurement();

    uint32_t voltage = ROUNDED_DIV(2 * adcSample * ADC_2V5_REF, ADC_RES);

    return voltage;
    
}