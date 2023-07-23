#include <time.h>

#include "em_cmu.h"
#include "em_emu.h"
#include "em_msc.h"
#include "em_rtc.h"
#include "em_usb.h"
#include "em_chip.h"
#include "em_core.h"
#include "em_usart.h"
#include "em_device.h"
#include "em_system.h"

#include "flash.h"
#include "radio.h"
#include "timer.h"
#include "pinouts.h"
#include "usbcallbacks.h"
#include "usbdescriptors.h"
#include "analogToDigitalConverter.h"
#include "accelerometer.h"

// LED pattern hyperparameters

// Flash interval before snapshot capturing has started
#define LED_INTERVAL_SECONDS                    5

/* Useful time constants */

#define MILLISECONDS_IN_SECOND                  1000

#define SECONDS_IN_MINUTE                       60
#define SECONDS_IN_HOUR                         (60 * SECONDS_IN_MINUTE)
#define SECONDS_IN_DAY                          (24 * SECONDS_IN_HOUR)

/* Clock frequencies */

#define HFXO_FREQ                               16368000
#define LFXO_FREQ                               32768

#define HFXO_FREQ_MARGIN                        10000

#define LFXO_CAL_COUNTS                         32

/* Hardware constants */

#define LFXO_TICKS_PER_SECOND                   1024

#define RTC_OVERFLOW                            (0x01 << 24)
#define RTC_DIV32                               5
#define RTC_COMP0                               0
#define RTC_COMP1                               1

#define MINIMUM_TICKS_TO_SLEEP                  128

#define TICKS_TO_CAPTURE                        17

#define DC_BOOST_BATTERY_THRESHOLD              350

/* Define Web USB constants */

#define USB_RECEIVE_BUFFER_SIZE                 64
#define USB_TRANSMIT_BUFFER_SIZE                128

#define USB_BOS_DESCRIPTOR                      0x0F
#define WEB_USB_REQUEST_TYPE                    0xC0

#define WEB_USB_URL_REQUEST                     0x01
#define WEB_USB_URL_INDEX                       0x0002

#define WEB_USB_MSFT_REQUEST                    0x02
#define WEB_USB_MSFT_INDEX                      0x0007

/* Firmware description contants */

#define FIRMWARE_VERSION_LENGTH                 3
#define FIRMWARE_DESCRIPTION_LENGTH             32

/* Flash bootloader constants */

#define FIRMWARE_CRC_POLY                       0x1021

#define FIRMWARE_LENGTH                         (48 * 1024)

#define FLASH_DATA_ADDRESS_OFFSET               FLASH_PAGES_PER_BLOCK
#define FLASH_FIRMWARE_ADDRESS_OFFSET           0

#define INTERNAL_FLASH_FIRMWARE_ADDRESS_OFFSET  0x4000
#define INTERNAL_FLASH_PAGE_SIZE                1024

/* Snapshot constants */

#define SNAPSHOT_BUFFER_LOCATION                0x20000800
#define SNAPSHOT_BUFFER_SIZE                    0x1800
#define PAGES_PER_SNAPSHOT                      (SNAPSHOT_BUFFER_SIZE / FLASH_PAGE_LENGTH)

/* Accelerometer constants */

#define ACCELERATION_MEASUREMENTS_PER_SNAPSHOT  20         

/* Useful macros */

#define USB_PLUGGED_IN                          (!GPIO_PinInGet(USB_SENSE_PORT, USB_SENSE_PIN))

#define MAX(a,b)                                (((a) > (b)) ? (a) : (b))

#define MIN(a,b)                                (((a) < (b)) ? (a) : (b))

#define ROUNDED_UP_DIV(a, b)                    (((a) + (b) - 1) / (b))

/* Web USB message enumerations */

typedef enum {
    SET_TIME_MESSAGE = 0x01,
    GET_STATUS_MESSAGE = 0x02,
    SET_FIRMWARE_INIT_MESSAGE = 0x03,
    SET_FIRMWARE_PAGE_MESSAGE = 0x04,
    GET_FIRMWARE_CRC_MESSAGE = 0x05,
    SET_FIRMWARE_FLASH_MESSAGE = 0x06,
    GET_METADATA_MESSAGE = 0x81,
    SET_RECORD_MESSAGE = 0x82,
    SET_SHUTDOWN_MESSAGE = 0x83,
    GET_SNAPSHOT_MESSAGE = 0x84
} usbMessageType_t;

/* Device state enumerations */

typedef enum {
  STATE_WILL_SHUTDOWN,                          // Device will shutdown after USB is unplugged
  STATE_WILL_RECORD,                            // Device will record snapshots after USB is unplugged
  STATE_ERASING                                 // Device is erasing the external flash
} deviceState_t;

/* Snapshot data structure */

#pragma pack(push, 1)

typedef struct {
  uint32_t time;                                // Unix timestamp
  uint32_t ticks: 10;                           // Ticks in second [0 to 1023]
  uint32_t pages: 2;                            // 2048B flash pages [1 to 3]
  int32_t temperature: 11;                      // d°C [-1024 to 1023]
  uint32_t batteryVoltage: 9;                   // cV [0 to 511]
} snapshotMetaData_t;

#pragma pack(pop)

/* Incoming USB message data structures */

#pragma pack(push, 1)

// SET_TIME_MESSAGE
// Set the time of the clock of the device.
// Echo message type.

typedef struct {
    uint32_t time;                                // Unix timestamp
    uint32_t ticks;                               // Ticks in second [0 to 1023]
} usbMessageSetTimeIn_t;

// SET_RECORD_MESSAGE
// Echo message type.
// Erase external flash memory, go to state STATE_ERASING while doing so.
// Go to state STATE_WILL_RECORD.
// After un-plugging, start recording snapshots in defined intervals after start time
// After end time, stop recording snapshots and go to state STATE_WILL_SHUTDOWN.

typedef struct {
    uint32_t measurementInterval;                 // Time between 2 snapshots in seconds
    uint32_t startTime;                           // Unix timestamp of first snapshot
    uint32_t endTime;                             // Unix timestamp of last snapshot
} usbMessageSetRecordIn_t;

#pragma pack(pop)

/* Outgoing USB message data structures */

#pragma pack(push, 1)

// GET_STATUS_MESSAGE
// Get general information from the device.

typedef struct {
    // Generic information
    uint32_t time;                                // Unix timestamp (device clock)
    uint32_t ticks;                               // Ticks in second [0 to 1023]
    uint32_t batteryVoltage;                      // Battery voltage in hundreds of volts
    uint64_t deviceID;
    uint8_t firmwareDescription[32];
    uint8_t firmwareVersion[3];
    uint32_t firmwareSize;                        // Length of firmware in bytes that is expected during a firmware update
    uint16_t firmwareChunkSize;                   // Length of individual firmware chunks in bytes in which the firmware is sent during an update
    // Device-specific information
    uint8_t state;                                // Device state
    uint16_t snapshotCount;                       // Number of stored snapshots
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t ctrlReg0;
    uint8_t ctrlReg1;
    uint8_t ctrlReg2;
    uint8_t ctrlReg3;
    uint8_t ctrlReg4;
    uint8_t ctrlReg5;
    uint8_t ctrlReg6;
} usbMessageGetStatusOut_t;

// GET_METADATA_MESSAGE
// Get meta data of next unread snapshot.
// The valid field indicates whether the subsequent data is valid or not.

typedef struct {
    uint8_t valid;                                // Whether the subsequent data is valid
    uint32_t time;                                // Unix timestamp (snapshot capture)
    uint32_t ticks;                               // Clock ticks (snapshot capture) [0-1023]
    uint32_t temperature;                         // Temperature in tenth of degrees kelvin
    uint32_t batteryVoltage;                      // Battery voltage in hundreds of volts
} usbMessageGetMetadataOut_t;

// GET_FIRMWARE_CRC_MESSAGE
// Calculate check value for CRC based on 48 KB firmware in external flash

typedef struct {
    uint16_t crc;                                 // Calculated check value for CRC
} usbMessageGetFirmwareCrcOut_t;

#pragma pack(pop)

/* Global USB buffers */

STATIC_UBUF(receiveBuffer, 2 * USB_RECEIVE_BUFFER_SIZE);

STATIC_UBUF(transmitBuffer, 2 * USB_TRANSMIT_BUFFER_SIZE);

/* Global device state variable */

static uint64_t timeOffset;

static deviceState_t state = STATE_WILL_SHUTDOWN;

static uint32_t measurementInterval = 20;

static uint32_t startTime = 0;

static uint32_t endTime = 0x7FFFFFFF;

static uint16_t snapshotCount = 0;

static uint32_t flash_nextAddress = FLASH_DATA_ADDRESS_OFFSET;

static uint8_t receivedFirmwarePages = FIRMWARE_LENGTH / FLASH_PAGE_LENGTH;

/* Global volatile event flags */

static volatile bool eventUSB;

static volatile bool eventRTC_Comp0;

static volatile bool eventRTC_Comp1;

static volatile bool shutdown = false;

static volatile bool eraseFlash = false;

static volatile bool loadFirmware = false;

/* Firmware version */

static uint8_t firmwareVersion[FIRMWARE_VERSION_LENGTH] = {0, 0, 2};

static uint8_t firmwareDescription[FIRMWARE_DESCRIPTION_LENGTH] = "SnapperGPS-Accelerometer";

/* Board version */

static bool legacyBoard = false;

// Accelerometer sensor variables

static int16_t x = 0;

static int16_t y = 0;

static int16_t z = 0;

static uint8_t ctrlReg[7] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Create acceleration arrays at same location as transmitBuffer
int16_t *accelerationsX = (int16_t*) transmitBuffer;
int16_t *accelerationsY = (int16_t*) transmitBuffer + ACCELERATION_MEASUREMENTS_PER_SNAPSHOT;
int16_t *accelerationsZ = (int16_t*) transmitBuffer + 2*ACCELERATION_MEASUREMENTS_PER_SNAPSHOT;

/* Interrupt handlers */

void RTC_IRQHandler(void) {

    // Get interrupt cause

    uint32_t interruptMask = RTC_IntGet();

    // Handle interrupt

    if (interruptMask & RTC_IFC_COMP0) eventRTC_Comp0 = true;

    if (interruptMask & RTC_IFC_COMP1) eventRTC_Comp1 = true;

    if (interruptMask & RTC_IFC_OF) timeOffset += RTC_OVERFLOW;

    // Clear the RTC interrupt flag

    RTC_IntClear(interruptMask);

}

void GPIO_EVEN_IRQHandler(void) {

    uint32_t interruptMask = GPIO_IntGet();

    // Handle interrupt

    if (interruptMask & (1 << USB_SENSE_PIN)) eventUSB = true;

    // Clear the GPIO interrupt flag

    GPIO_IntClear(interruptMask);

}

/* LED functions */

static void enableRedLED(bool enable) {

    GPIO_PinModeSet(RED_LED_PORT, RED_LED_PIN, gpioModePushPull, enable);

}

static void enableGreenLED(bool enable) {

    GPIO_PinModeSet(GREEN_LED_PORT, GREEN_LED_PIN, gpioModePushPull, enable);

}

/* DC boost function */

void enableDCBoost() {

    GPIO_PinModeSet(DC_BOOST_EN_PORT, DC_BOOST_EN_PIN, gpioModeInputPull, 1);

}

void disableDCBoost() {

    GPIO_PinModeSet(DC_BOOST_EN_PORT, DC_BOOST_EN_PIN, gpioModeInputPull, 0);

}

/* Time handling functions */

static void getTime(uint32_t *time, uint32_t *ticks, uint64_t *counter) {

    uint64_t rawCounter = RTC_CounterGet();

    uint64_t updatedCounter = rawCounter + timeOffset;

    if (time) *time = (uint32_t)(updatedCounter / LFXO_TICKS_PER_SECOND);

    if (ticks) *ticks = (uint32_t)(updatedCounter % LFXO_TICKS_PER_SECOND);

    if (counter) *counter = rawCounter;

}

static void setTime(uint32_t time, uint32_t ticks) {

    uint64_t requiredCounter = (uint64_t)time * LFXO_TICKS_PER_SECOND + (uint64_t)ticks;

    timeOffset = requiredCounter - (uint64_t)RTC_CounterGet();

}

/* Callback which provides the WebUSB specific descriptors */

int setupCmd(const USB_Setup_TypeDef *setup) {

    if (setup->Type == USB_SETUP_TYPE_STANDARD && setup->bRequest == GET_DESCRIPTOR && setup->wValue >> 8 == USB_BOS_DESCRIPTOR) {

        USBD_Write(0, (void*)BOS_Descriptor, SL_MIN(sizeof(BOS_Descriptor), setup->wLength), NULL);

        return USB_STATUS_OK;

    }

    if (setup->bmRequestType == WEB_USB_REQUEST_TYPE) {

        if (setup->bRequest == WEB_USB_URL_REQUEST && setup->wIndex == WEB_USB_URL_INDEX) {

            USBD_Write(0, (void*)URL_Descriptor, SL_MIN(sizeof(URL_Descriptor), setup->wLength), NULL);

            return USB_STATUS_OK;

        }

        if (setup->bRequest == WEB_USB_MSFT_REQUEST && setup->wIndex == WEB_USB_MSFT_INDEX) {

            USBD_Write(0, (void*)MICROSOFT_Descriptor, SL_MIN(sizeof(MICROSOFT_Descriptor), setup->wLength), NULL);

            return USB_STATUS_OK;

        }

    }

    return USB_STATUS_REQ_UNHANDLED;

}

/* Callback to start the USB reading process when the device is configured */

void stateChange(USBD_State_TypeDef oldState, USBD_State_TypeDef newState) {

    if (newState == USBD_STATE_CONFIGURED) {

        USBD_Read(WEBUSB_EP_OUT, receiveBuffer, USB_RECEIVE_BUFFER_SIZE, dataReceivedWebUSBCallback);

    }

}

/* Callback on completion of data send. Used to request next read */

int dataSentWebUSBCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining) {

    if (receivedFirmwarePages < FIRMWARE_LENGTH / FLASH_PAGE_LENGTH) {

        // Copy data in firmware buffer

        USBD_Read(WEBUSB_EP_OUT, (uint8_t*)SNAPSHOT_BUFFER_LOCATION, FLASH_PAGE_LENGTH, dataReceivedWebUSBCallback);

        receiveBuffer[0] = SET_FIRMWARE_PAGE_MESSAGE;

    } else {

        // Copy data to receive buffer

        USBD_Read(WEBUSB_EP_OUT, receiveBuffer, USB_RECEIVE_BUFFER_SIZE, dataReceivedWebUSBCallback);

    }

    return USB_STATUS_OK;

}

/* CRC update function */

static uint16_t updateCRC(uint16_t crc, int incr) {

    uint16_t xor = crc >> 15;

    uint16_t out = crc << 1;

    if (incr) out++;

    if (xor) out ^= FIRMWARE_CRC_POLY;

    return out;

}

/* Callback on receipt of message from the USB host */

int dataReceivedWebUSBCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining) {

    /* Write default returned message */

    memset(transmitBuffer, 0, USB_TRANSMIT_BUFFER_SIZE);

    transmitBuffer[0] = receiveBuffer[0];

    /* Respond appropriately to message */

    switch((usbMessageType_t)receiveBuffer[0]) {

        case SET_TIME_MESSAGE: ;

            usbMessageSetTimeIn_t* timeData = (usbMessageSetTimeIn_t*)(receiveBuffer + 1);

            setTime(timeData->time, timeData->ticks);

            break;

        case GET_STATUS_MESSAGE:;

            // Define status data struct over transmit buffer

            usbMessageGetStatusOut_t *statusMsg = (usbMessageGetStatusOut_t*)(transmitBuffer + 1);

            // Get current device time with millisecond precission

            getTime(&statusMsg->time, &statusMsg->ticks, NULL);

            // Measure battery voltage in centi-volts

            AnalogToDigitalConverter_enable();
            AnalogToDigitalConverter_enableBatteryMeasurement(legacyBoard ? PULL_LOW : PULL_HIGH);
            statusMsg->batteryVoltage = AnalogToDigitalConverter_measureBatteryVoltage();
            AnalogToDigitalConverter_disableBatteryMeasurement();
            AnalogToDigitalConverter_disable();

            statusMsg->x = x;
            statusMsg->y = y;
            statusMsg->z = z;
            statusMsg->ctrlReg0 = ctrlReg[0];
            statusMsg->ctrlReg1 = ctrlReg[1];
            statusMsg->ctrlReg2 = ctrlReg[2];
            statusMsg->ctrlReg3 = ctrlReg[3];
            statusMsg->ctrlReg4 = ctrlReg[4];
            statusMsg->ctrlReg5 = ctrlReg[5];
            statusMsg->ctrlReg6 = ctrlReg[6];

            // Assemble message

            statusMsg->state = state;
            statusMsg->deviceID = SYSTEM_GetUnique();
            statusMsg->snapshotCount = snapshotCount;
            memcpy(&statusMsg->firmwareDescription, firmwareDescription, FIRMWARE_DESCRIPTION_LENGTH);
            memcpy(&statusMsg->firmwareVersion, firmwareVersion, FIRMWARE_VERSION_LENGTH);
            statusMsg->firmwareSize = FIRMWARE_LENGTH;
            statusMsg->firmwareChunkSize = FLASH_PAGE_LENGTH;

            break;

        case GET_METADATA_MESSAGE: ;

            // Address of destination array
            // (where to write the snapshot that is taken from external flash memory)

            uint8_t *address = (uint8_t*)SNAPSHOT_BUFFER_LOCATION;

            // Number of pages to read
            // (update after meta data is read)

            uint32_t pages = 1;

            // If data is valid

            uint8_t valid = true;

            // Iterate over every page to read for this snapshot (go forwards)

            for (uint32_t i = 0; i < pages; ++i) {

                // Check if no unread page exists

                if (flash_nextAddress >= (FLASH_NUM_PAGES))  valid = false;

                // Check if flash is busy

                while (Flash_isBusy());

                // Read i-th page of flash

                FlashResult_t flashMsg = Flash_readPage(flash_nextAddress, address);

                // Check if flash is busy

                while (Flash_isBusy());

                // Check if this is page with meta data

                if (i == 0) {

                    // Remember if data could not be read

                    if (flashMsg != FLASH_SUCCESS)  valid = false;

                    // Create struct that holds meta data

                    snapshotMetaData_t* ptr_metaData = (snapshotMetaData_t*) address;

                    // Check if 0xFFFFFFFF has been read (indicates empty page, no more data in flash to read)

                    if (ptr_metaData->time == 0xFFFFFFFF)  valid = false;

                    if (valid) {

                        // Number of pages for this snapshot (can be read from ptr_metaData->pages, too)

                        pages = PAGES_PER_SNAPSHOT;

                        // Check if full snapshot is available

                        if (flash_nextAddress + pages > FLASH_NUM_PAGES)  valid = false;

                    }

                    // Define meta data struct over transmit buffer

                    usbMessageGetMetadataOut_t *metadataMsg = (usbMessageGetMetadataOut_t*)(transmitBuffer + 1);

                    metadataMsg->valid = valid;
                    metadataMsg->time = ptr_metaData->time;
                    metadataMsg->ticks = ptr_metaData->ticks;
                    metadataMsg->temperature = ptr_metaData->temperature+1024;
                    metadataMsg->batteryVoltage = ptr_metaData->batteryVoltage;

                }

                if (valid) {

                    // Next unread page in external flash

                    ++flash_nextAddress;

                    // Next unfilled area in buffer

                    address += FLASH_PAGE_LENGTH * sizeof(uint8_t);

                } else {

                    // Reset address of next page to start (include offset for area reserved for firmware)

                    flash_nextAddress = FLASH_DATA_ADDRESS_OFFSET;

                }

            }

            break;

        case SET_RECORD_MESSAGE: ;

            usbMessageSetRecordIn_t* configMsg = (usbMessageSetRecordIn_t*)(receiveBuffer + 1);

            // Time between 2 consecutive snapshots / measurements [s]

            measurementInterval = configMsg->measurementInterval;

            // Unix timestamp of first snapshot

            startTime = configMsg->startTime;

            // Unix timestamp of last snapshot

            endTime = configMsg->endTime;

            // Erase external flash memory next

            eraseFlash = true;

            break;

        case SET_SHUTDOWN_MESSAGE:

           shutdown = true;

            break;

        case GET_SNAPSHOT_MESSAGE:

            // Turn off red LED

            enableRedLED(false);

            // Send snapshot directly from the snapshot buffer

            USBD_Write(WEBUSB_EP_IN,
                       ((uint8_t*)SNAPSHOT_BUFFER_LOCATION)+sizeof(snapshotMetaData_t),
                       SNAPSHOT_BUFFER_SIZE-sizeof(snapshotMetaData_t),
                       dataSentWebUSBCallback);

            // Turn off red LED

            enableRedLED(false);

            return USB_STATUS_OK;

        case SET_FIRMWARE_INIT_MESSAGE:

            // Turn on red LED

            enableRedLED(true);

            // Check if flash is busy

            while (Flash_isBusy());

            // Make blocks writable

            Flash_removeProtection();

            // Check if flash is busy

            while (Flash_isBusy());

            // Erase flash if it is not busy

            Flash_blockErase(FLASH_FIRMWARE_ADDRESS_OFFSET);

            // Check if flash is busy

            while (Flash_isBusy());

            // Receive firmware pages next

            receivedFirmwarePages = 0;

            // Turn off red LED

            enableRedLED(false);

            break;

        case SET_FIRMWARE_PAGE_MESSAGE:

            // Write a single firmware page to external flash memory
            // Firmware can be written to last 128 KB block of external flash memory
            // Data is written to the next empty page

            // Turn on red LED

            enableRedLED(true);

            // Check if flash is busy

            while (Flash_isBusy());

            // Write page to flash

            Flash_writePage(FLASH_FIRMWARE_ADDRESS_OFFSET + receivedFirmwarePages, (uint8_t*)SNAPSHOT_BUFFER_LOCATION);

            // Increment to next page

            receivedFirmwarePages++;

            // Turn off red LED

            enableRedLED(false);

            break;

        case GET_FIRMWARE_CRC_MESSAGE: ;

            // Turn on red LED

            enableRedLED(true);

            // Calculate check value for CRC based on 48 KB firmware in external flash

            usbMessageGetFirmwareCrcOut_t crcMsg;

            crcMsg.crc = 0;

            // Check if flash is busy

            while (Flash_isBusy());

            // Enabled continuous reading from flash

            Flash_enableContinuousRead(FLASH_FIRMWARE_ADDRESS_OFFSET);

            for (uint32_t i = 0; i < FIRMWARE_LENGTH; i += 1) {

                uint8_t data = Flash_readContinuousByte();

                for (uint16_t i = 0x80; i > 0; i >>= 1) {
                    crcMsg.crc = updateCRC(crcMsg.crc, data & i);
                }

            }

            for (uint16_t i = 0; i < 16; i++) {
                crcMsg.crc = updateCRC(crcMsg.crc, 0);
            }

            Flash_disableContinuousRead();

            // Put CRC value into USB buffer ready to be sent

            *(usbMessageGetFirmwareCrcOut_t*)(&transmitBuffer[1]) = crcMsg;

            // Turn off red LED

            enableRedLED(false);

            break;

        case SET_FIRMWARE_FLASH_MESSAGE:

            if (receivedFirmwarePages == FIRMWARE_LENGTH / FLASH_PAGE_LENGTH) {

                // Turn on red LED

                enableRedLED(true);

                // Set the flag

                loadFirmware = true;

            }

            break;

        default:

            break;

    }

    // Send the response */

    USBD_Write(WEBUSB_EP_IN, transmitBuffer, USB_TRANSMIT_BUFFER_SIZE, dataSentWebUSBCallback);

    return USB_STATUS_OK;

}

/* Function held in SRAM to write firmware to internal flash */

SL_RAMFUNC_DEFINITION_BEGIN
static void __attribute__ ((noinline)) writeFirmwareToInternalFlash() {

    for (uint32_t i = 0; i < FIRMWARE_LENGTH; i += INTERNAL_FLASH_PAGE_SIZE) {

        uint32_t *destination = (uint32_t*)(INTERNAL_FLASH_FIRMWARE_ADDRESS_OFFSET + i);

        for (uint32_t j = 0; j < INTERNAL_FLASH_PAGE_SIZE; j += 1) {

            uint8_t *source = (uint8_t*)(SNAPSHOT_BUFFER_LOCATION + j);

            // Read the next byte from the flash

            while (!(FLASH_USART->STATUS & USART_STATUS_TXBL));

            FLASH_USART->TXDATA = (uint32_t)0xFF;

            while (!(FLASH_USART->STATUS & USART_STATUS_TXC));

            // Write the byte to the snapshot buffer

            *source = (uint8_t)FLASH_USART->RXDATA;

        }

        // Unlock the internal flash for erasing and writing

        MSC->LOCK = MSC_UNLOCK_CODE;

        MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

        // Erase the internal flash page

        MSC->ADDRB = (uint32_t)destination;

        MSC->WRITECMD = MSC_WRITECMD_LADDRIM;

        MSC->WRITECMD = MSC_WRITECMD_ERASEPAGE;

        while (MSC->STATUS & MSC_STATUS_BUSY);

        // Write the internal flash page

        for (uint32_t j = 0; j < INTERNAL_FLASH_PAGE_SIZE; j += 4) {

            uint32_t *source = (uint32_t*)(SNAPSHOT_BUFFER_LOCATION + j);

            MSC->WDATA = *source;

            MSC->WRITECMD = MSC_WRITECMD_WRITEONCE;

            while (MSC->STATUS & MSC_STATUS_BUSY);

        }

        // Lock the internal flash

        MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;

        MSC->LOCK = 0;

    }

    // Reset to start the new firmware

    NVIC_SystemReset();

}
SL_RAMFUNC_DEFINITION_END

/* HFXO measurement function */

static uint32_t measureHFXO() {

    CMU->CALCNT = LFXO_CAL_COUNTS - 1;

    CMU->CALCTRL = CMU_CALCTRL_UPSEL_HFXO | CMU_CALCTRL_DOWNSEL_LFXO;

    CMU->CMD |= CMU_CMD_CALSTART;

    while (CMU->STATUS & CMU_STATUS_CALBSY);

    return LFXO_FREQ / LFXO_CAL_COUNTS * CMU->CALCNT;

}

/* Main function */

int main(void) {

    CHIP_Init();

    // Enable high frequency peripherals

    CMU_ClockEnable(cmuClock_HFPER, true);

    // Enable GPIO

    CMU_ClockEnable(cmuClock_GPIO, true);

    // Enable LFXO and low energy domain

    CMU_ClockEnable(cmuClock_CORELE, true);

    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

    // Enable DC boost

    enableDCBoost();

    // Enable timer

    Timer_enable();

    // Enable LED pins

    GPIO->ROUTE &= ~(GPIO_ROUTE_SWDIOPEN | GPIO_ROUTE_SWCLKPEN);

    // Enable RTC

    CMU_ClockEnable(cmuClock_RTC, true);

    CMU->LFAPRESC0 = RTC_DIV32;

    // Initialise real-time counter

    RTC_Init_TypeDef rtcInit = RTC_INIT_DEFAULT;

    rtcInit.comp0Top = false;

    RTC_Init(&rtcInit);

    // Check board version (battery voltage measurement circuit differs)

    AnalogToDigitalConverter_enable();
    // Measure voltage without battery voltage measurement enabled
    uint32_t voltageDisabled = AnalogToDigitalConverter_measureBatteryVoltage();
    // Enable battery voltage measurement assuming legacy board
    AnalogToDigitalConverter_enableBatteryMeasurement(PULL_LOW);
    // Check if voltage measurement increases (by more than threshold)
    uint32_t legacyBoardThreshold = 15;  // Centi-volts
    legacyBoard = AnalogToDigitalConverter_measureBatteryVoltage() >= voltageDisabled + legacyBoardThreshold;
    AnalogToDigitalConverter_disableBatteryMeasurement();
    AnalogToDigitalConverter_disable();

    // Enable interrupt on USB_SENSE to wake from generate interrupt on unplug and to wake from EM3

    GPIO_PinModeSet(USB_SENSE_PORT, USB_SENSE_PIN, gpioModeInputPullFilter, 1);

    GPIO_IntConfig(USB_SENSE_PORT, USB_SENSE_PIN, true, true, true);

    NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);

    NVIC_EnableIRQ(GPIO_EVEN_IRQn);

    // Initialise USB

    USBD_Init(&usbInitStruct);

    if (USB_PLUGGED_IN) {

        // Count the number of snapshots in the external flash memory

        // Turn on both LED

        enableRedLED(true);
        enableGreenLED(true);

        // Power up and enable flash

        Flash_powerOn();

        Timer_delayMilliseconds(1);

        Flash_enableInterface();

        // Check for board with 1-GBit external flash

        Flash_init();

        // Address of next unread page in flash

        flash_nextAddress = FLASH_DATA_ADDRESS_OFFSET;

        // Declare struct that holds meta data

        snapshotMetaData_t metaData;

        uint8_t valid = true;

        // Counting existing snapshots on flash

        snapshotCount = 0;

        while (flash_nextAddress < FLASH_NUM_PAGES && valid) {

            // Check if flash is busy

            while (Flash_isBusy());

            // Read i-th page of flash

            Flash_readBytes(flash_nextAddress, (uint8_t*)(&metaData), sizeof(snapshotMetaData_t));

            // Check if flash is busy

            while (Flash_isBusy());

            // Check if 0xFFFFFFFF has been read (indicates empty page, no more data in flash to read)

            if (metaData.time == 0xFFFFFFFF) {

                valid = false;

            } else {

                ++snapshotCount;

                // Increment address to next unread page in external flash

                flash_nextAddress += PAGES_PER_SNAPSHOT;

            }

        }

        flash_nextAddress = FLASH_DATA_ADDRESS_OFFSET;

        // Configure accelerometer

        Accelerometer_enableInterface();

        // Check if accelerometer is available
        if (Accelerometer_isAvailable()) {

            Accelerometer_enableLowPowerMode();

        }

        Accelerometer_disableInterface();

        // Turn off red LED

        enableRedLED(false);

        // Main loop whilst USB plugged in

        while (USB_PLUGGED_IN && !loadFirmware) {

            if (eraseFlash) {

                // Erase flash

                state = STATE_ERASING;

                // Turn on red LED

                enableRedLED(true);

                // Check if flash is busy

                while (Flash_isBusy());

                // Make blocks writebale

                Flash_removeProtection();

                // Check if flash is busy

                while (Flash_isBusy());

                // Erase flash if it is not busy

                Flash_erase();

                // Check if flash is busy

                while (Flash_isBusy()) Timer_delayMilliseconds(100);

                // Can write new snapshots from beginning of reserved area

                flash_nextAddress = FLASH_DATA_ADDRESS_OFFSET;

                eraseFlash = false;

                snapshotCount = 0;

                state = STATE_WILL_RECORD;

                // Turn off red LED

                enableRedLED(false);

            }

            if (shutdown) {

                // Device will shutdown after unplugging

                state = STATE_WILL_SHUTDOWN;

                shutdown = false;

            }

            // Measure acceleration

            enableRedLED(true);

            Accelerometer_enableInterface();

            // Check if accelerometer is available
            if (Accelerometer_isAvailable()) {

                Accelerometer_selectDataRate(1);

                // Wait until accelerometer data is available
                bool dataAvailable = Accelerometer_isNewDataAvailable();
                while (!dataAvailable) {
                    Timer_delayMicroseconds(500);
                    dataAvailable = Accelerometer_isNewDataAvailable();
                }

                enableGreenLED(true);

                Accelerometer_readXYZ(&x, &y, &z);

                Accelerometer_readCtrlReg(ctrlReg);

                Accelerometer_selectDataRate(0);

            } else {

                x = 0;
                y = 0;
                z = 0;
                for (uint8_t i = 0; i < 7; ++i) {
                    ctrlReg[i] = 0;
                }

            }

            Accelerometer_disableInterface();

            enableRedLED(false);

            Timer_delayMilliseconds(200);

            enableGreenLED(false);

            Timer_delayMilliseconds(100);

            // EMU_EnterEM1();

            // Timer_delayMilliseconds(500);

        }

        // Check if flash is busy

        while (Flash_isBusy());

        // Load the firmware if that was requested

        if (loadFirmware) {

            // Wait for last USB message to send

            Timer_delayMilliseconds(20);

            // Stop the USB interface

            USBD_Stop();

            // Enable continuous reading from flash

            Flash_enableContinuousRead(FLASH_FIRMWARE_ADDRESS_OFFSET);

            // Disable interrupts

            CORE_CriticalDisableIrq();

            // Call SRAM function to write firmware to internal flash
            // This SRAM function call does not return

            writeFirmwareToInternalFlash();

        }

        // USB unplugged

        enableGreenLED(false);

        USBD_Stop();

        // Turn off the flash

        Flash_disableInterface();

        Flash_powerOff();

        // Disable DC boost

        Timer_delayMilliseconds(20);

        disableDCBoost();

        if (state == STATE_WILL_RECORD) {

            // Flash green LED to indicate initialized recording

            for (uint32_t i = 0; i < 10; i += 1) {
                enableGreenLED(true);
                Timer_delayMilliseconds(100);
                enableGreenLED(false);
                Timer_delayMilliseconds(100);
            }

            // Get the current time and the current real-time counter compare register value

            uint64_t counter;

            uint32_t time, ticks;

            getTime(&time, &ticks, &counter);

            // Calculate the current time rounded up to the next integer measurement interval

            time_t rawTime = time;

            struct tm *tm = gmtime(&rawTime);

            uint32_t secondsSinceStartOfDay = SECONDS_IN_HOUR * tm->tm_hour + SECONDS_IN_MINUTE * tm->tm_min + tm->tm_sec;

            int64_t currentTimeRoundedUpToMeasurementInterval = (int64_t)time - (int64_t)secondsSinceStartOfDay + (int64_t)measurementInterval * (int64_t)ROUNDED_UP_DIV(secondsSinceStartOfDay, measurementInterval);

            // Calculate the number of ticks until the later of the start time and the current time rounded up to the next integer measurement interval

            int64_t ticksUntilFirstInterrupt = MAX(currentTimeRoundedUpToMeasurementInterval, (int64_t)startTime) * LFXO_TICKS_PER_SECOND - (int64_t)time * LFXO_TICKS_PER_SECOND - (int64_t)ticks - TICKS_TO_CAPTURE;

            // Wait an extra measurement interval if there is not enough time to sleep

            if (ticksUntilFirstInterrupt < MINIMUM_TICKS_TO_SLEEP) ticksUntilFirstInterrupt += (int64_t)measurementInterval * LFXO_TICKS_PER_SECOND;

            // Increment the real-time counter compare register

            uint64_t compare = counter + ticksUntilFirstInterrupt;

            // Remember higher bits of the 24-bit real-time counter compare register to count down to the real start time

            uint64_t delayedStartCount = compare >> 24;

            // Enable interrupt using real-time counter with compare register 0

            RTC_CompareSet(RTC_COMP0, compare);

            RTC_IntEnable(RTC_IEN_COMP0);

            // Enable interrupt to regularly flash LED while waiting using real-time counter with compare register 1

            RTC_CompareSet(RTC_COMP1, counter + LED_INTERVAL_SECONDS * LFXO_TICKS_PER_SECOND);

            RTC_IntEnable(RTC_IEN_COMP1);

            NVIC_ClearPendingIRQ(RTC_IRQn);

            NVIC_EnableIRQ(RTC_IRQn);

            // Initialize real-time counter interrupt event flags

            eventRTC_Comp0 = false;

            eventRTC_Comp1 = false;

            // Disable timer

            Timer_disable();

            // Counter for number of measurements
            // A snapshot is triggered whenever this counter reaches ACCELERATION_MEASUREMENTS_PER_SNAPSHOT
            uint8_t accelerationsCount = ACCELERATION_MEASUREMENTS_PER_SNAPSHOT - 1;

            // Remember if recording has started already
            bool started = false;

            // Record snapshots as long as USB is not plugged in, end date is not reached, and flash memory is not full

            while (!USB_PLUGGED_IN && state == STATE_WILL_RECORD) {

                if (eventRTC_Comp0) {

                    // Time interval since last measurement has passed

                    if (delayedStartCount == 0) {

                        getTime(&time, &ticks, NULL);

                        if (!started) {

                            // Disable interrupts; so, LED does not flash every 5 s anymore

                            RTC_IntDisable(RTC_IEN_COMP1);

                            // Remember actual start time

                            startTime = time;

                            // Remember that recording started

                            started = true;

                        }

                        // Check if end time is reached or flash memory is full

                        if (time + measurementInterval >= endTime || flash_nextAddress + 2*3 > FLASH_NUM_PAGES) {

                            state = STATE_WILL_SHUTDOWN;

                            // Disable interrupts so no more snapshots are acquired

                            RTC_IntDisable(RTC_IEN_COMP0);

                            NVIC_ClearPendingIRQ(RTC_IRQn);

                            NVIC_DisableIRQ(RTC_IRQn);

                        } else {

                            // Update the real-time counter compare register for the next interrupt
                            // An interrupt triggers an acceleration measurement

                            uint32_t compare = RTC_CompareGet(RTC_COMP0) + (measurementInterval * LFXO_TICKS_PER_SECOND) / ACCELERATION_MEASUREMENTS_PER_SNAPSHOT;

                            RTC_CompareSet(RTC_COMP0, compare);

                        }

                        /* Enable DC boost */

                        enableDCBoost();

                        // Enable timer

                        Timer_enable();

                        // Read acceleration

                        Accelerometer_enableInterface();

                        // Check if accelerometer is available
                        if (Accelerometer_isAvailable()) {

                            Accelerometer_selectDataRate(1);

                            // Wait until accelerometer data is available
                            bool dataAvailable = Accelerometer_isNewDataAvailable();
                            while (!dataAvailable) {
                                Timer_delayMicroseconds(500);
                                dataAvailable = Accelerometer_isNewDataAvailable();
                            }

                            Accelerometer_readXYZ(&x, &y, &z);

                            Accelerometer_selectDataRate(0);

                        } else {

                            x = 0;
                            y = 0;
                            z = 0;

                        }

                        Accelerometer_disableInterface();

                        // Append new accelerations to arrays
                        accelerationsX[accelerationsCount] = x;
                        accelerationsY[accelerationsCount] = y;
                        accelerationsZ[accelerationsCount] = z;
                        // Increment counter
                        ++accelerationsCount;

                        // Check if arrays are full

                        if (accelerationsCount >= ACCELERATION_MEASUREMENTS_PER_SNAPSHOT) {

                            // Enable the ADC

                            AnalogToDigitalConverter_enable();

                            // Read ambient temperature in centi-degrees Celsius

                            int32_t temperature = AnalogToDigitalConverter_measureTemperature();

                            AnalogToDigitalConverter_enableBatteryMeasurement(legacyBoard ? PULL_LOW : PULL_HIGH);

                            // Read battery voltage in centi-volts

                            uint32_t batteryVoltage = AnalogToDigitalConverter_measureBatteryVoltage();

                            AnalogToDigitalConverter_disableBatteryMeasurement();

                            // Disable the ADC

                            AnalogToDigitalConverter_disable();

                            // Power up radio

                            Radio_powerOn();

                            Radio_enableHFXOInput();

                            // Wait short period for radio to stabilize

                            Timer_delayMilliseconds(10);

                            // Check that the HFXO is running

                            uint32_t frequency =  measureHFXO();

                            bool frequencyGood = frequency > HFXO_FREQ - HFXO_FREQ_MARGIN && frequency < HFXO_FREQ + HFXO_FREQ_MARGIN;

                            if (frequencyGood) {

                                // Switch HF clock to HFXO

                                CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

                                // Get timestamp for snapshot

                                getTime(&time, &ticks, NULL);

                                // Capture snapshot

                                Radio_captureSnapshot((uint8_t*)SNAPSHOT_BUFFER_LOCATION, SNAPSHOT_BUFFER_SIZE);

                                // Switch HF clock to HFRCO

                                CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);

                            } else {

                                // Set snapshot to default value 0

                                memset((uint8_t*)SNAPSHOT_BUFFER_LOCATION, 0, SNAPSHOT_BUFFER_SIZE);

                                // Get timestamp

                                getTime(&time, &ticks, NULL);

                            }

                            // Power off the radio

                            Radio_disableHFXOInput();

                            Radio_powerOff();

                            if (frequencyGood) {

                                // TODO: remove (Flash green LED once)

                                enableGreenLED(true);
                                Timer_delayMilliseconds(10);
                                enableGreenLED(false);

                            } else {

                                // TODO: remove (Flash red LED once)

                                enableRedLED(true);
                                Timer_delayMilliseconds(10);
                                enableRedLED(false);

                            }

                            // Power up and enable flash

                            Flash_powerOn();
                            Timer_delayMilliseconds(1);
                            Flash_enableInterface();

                            // Make blocks writebale

                            while (Flash_isBusy());

                            Flash_removeProtection();

                            // Create struct that holds meta data and overwrite the first few bytes of the snapshot

                            snapshotMetaData_t *metaData = (snapshotMetaData_t*)SNAPSHOT_BUFFER_LOCATION;

                            metaData->time = time;
                            metaData->ticks = ticks;
                            metaData->pages = PAGES_PER_SNAPSHOT;
                            metaData->temperature = temperature;
                            metaData->batteryVoltage = batteryVoltage;

                            // Address of source array

                            uint8_t* address = (uint8_t*)SNAPSHOT_BUFFER_LOCATION;

                            // // Overwrite 1st element with accelerations size
                            address[sizeof(snapshotMetaData_t)] = accelerationsCount;

                            // Overwrite 2nd, 3rd, ... elements with accelerations
                            for (uint8_t i = 0; i < accelerationsCount; ++i) {
                                address[sizeof(snapshotMetaData_t) + 1 + i * sizeof(int16_t)] = accelerationsX[i];
                                address[sizeof(snapshotMetaData_t) + 1 + (ACCELERATION_MEASUREMENTS_PER_SNAPSHOT + i) * sizeof(int16_t)] = accelerationsY[i];
                                address[sizeof(snapshotMetaData_t) + 1 + (2 * ACCELERATION_MEASUREMENTS_PER_SNAPSHOT + i) * sizeof(int16_t)] = accelerationsZ[i];
                            }

                            // Reset accelerations count
                            accelerationsCount = 0;

                            // Iterate over every page to write for this snapshot

                            for (uint32_t i = 0; i < PAGES_PER_SNAPSHOT; ++i) {

                                // Check if there is free memory in flash

                                if (flash_nextAddress < FLASH_NUM_PAGES) {

                                    // Check if flash is busy

                                    while (Flash_isBusy());

                                    // Write page to flash

                                    Flash_writePage(flash_nextAddress, address);

                                    // Increment address to next page

                                    ++flash_nextAddress;

                                    address += FLASH_PAGE_LENGTH * sizeof(uint8_t);

                                }

                            }

                            // Check if flash is busy

                            while (Flash_isBusy());

                            // Turn off flash

                            Flash_disableInterface();

                            Flash_powerOff();

                            // Disable DC boost

                            if (batteryVoltage < DC_BOOST_BATTERY_THRESHOLD) Timer_delayMilliseconds(20);

                        }

                        disableDCBoost();

                        // Disable timer if another snapshot will be recorded

                        if (state == STATE_WILL_RECORD) Timer_disable();

                    } else {

                        --delayedStartCount;

                    }

                    // Reset interrupt event flag

                    eventRTC_Comp0 = false;

                } else {

                    if (eventRTC_Comp1) {

                        // Still waiting to record first snapshot

                        // Update the real-time counter compare register for the next interrupt

                        RTC_CompareSet(RTC_COMP1, RTC_CompareGet(RTC_COMP1)
                                                  + LED_INTERVAL_SECONDS * LFXO_TICKS_PER_SECOND);

                        // Flash LEDs

                        Timer_enable();

                        enableGreenLED(true);
                        enableRedLED(true);
                        Timer_delayMilliseconds(10);
                        enableGreenLED(false);
                        enableRedLED(false);

                        Timer_disable();

                        // Reset interrupt flag

                        eventRTC_Comp1 = false;

                    }

                }

                // Enter EM3 if recording more snapshots

                if (state == STATE_WILL_RECORD)  EMU_EnterEM3(false);

            }

        }

    }

    if (state == STATE_WILL_SHUTDOWN) {

        // Flash red LED to indicate power down

        for (uint32_t i = 0; i < 10; i += 1) {
            enableRedLED(true);
            Timer_delayMilliseconds(100);
            enableRedLED(false);
            Timer_delayMilliseconds(100);
        }

        // Disable timer

        Timer_disable();

        // Enter EM4

        GPIO->EM4WUEN = GPIO_EM4WUEN_EM4WUEN_F2;

        GPIO->CTRL = GPIO_CTRL_EM4RET;

        GPIO->CMD = GPIO_CMD_EM4WUCLR;

        EMU_EnterEM4();

        // Enter EM3 if something goes wrong while entering EM4

        CMU_ClockEnable(cmuClock_CORELE, false);

        CMU_OscillatorEnable(cmuOsc_LFXO, false, false);

        while (!USB_PLUGGED_IN) EMU_EnterEM3(false);

    }

    // Reset on USB plug-in

    NVIC_SystemReset();

}
