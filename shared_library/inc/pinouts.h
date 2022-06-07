/****************************************************************************
 * pinouts.h
 * Snapper GPS
 * November 2020
 *****************************************************************************/

#ifndef __PINOUTS_H
#define __PINOUTS_H

/* LED */

#define RED_LED_PORT            gpioPortF
#define RED_LED_PIN             0

#define GREEN_LED_PORT          gpioPortF
#define GREEN_LED_PIN           1

/* Debug serial port */

#define SERIAL_TX_PORT          gpioPortF
#define SERIAL_TX_PIN           0

#define SERIAL_RX_PORT          gpioPortF
#define SERIAL_RX_PIN           1

/* USB sense */

#define USB_SENSE_PORT          gpioPortF
#define USB_SENSE_PIN           2

/* Flash */

#define FLASH_VDD_EN_PORT       gpioPortB
#define FLASH_VDD_EN_PIN        11

#define FLASH_MOSI_PORT         gpioPortE
#define FLASH_MOSI_PIN          10

#define FLASH_MISO_PORT         gpioPortE
#define FLASH_MISO_PIN          11

#define FLASH_SCLK_PORT         gpioPortE
#define FLASH_SCLK_PIN          12

#define FLASH_CS_PORT           gpioPortE
#define FLASH_CS_PIN            13

/* Radio */

#define RADIO_VDD_EN_PORT       gpioPortC
#define RADIO_VDD_EN_PIN        0

#define RADIO_CLK_PORT          gpioPortB
#define RADIO_CLK_PIN           14

#define RADIO_SIGN_PORT         gpioPortC
#define RADIO_SIGN_PIN          1

/* GPIO */

#define GPIO1_PORT              gpioPortA
#define GPIO1_PIN               0

#define GPIO2_PORT              gpioPortA
#define GPIO2_PIN               1

#define GPIO3_PORT              gpioPortA
#define GPIO3_PIN               2

/* Battery */

#define VBAT_SENSE_EN_PORT      gpioPortD
#define VBAT_SENSE_EN_PIN       6

#define VBAT_SENSE_PORT         gpioPortD
#define VBAT_SENSE_PIN          7

/* Voltage boost */

#define DC_BOOST_EN_PORT        gpioPortD
#define DC_BOOST_EN_PIN         5

#endif /* __PINOUTS_H */