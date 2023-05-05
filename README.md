# snappergps-firmware

This is the standard firmware for a SnapperGPS receiver.
The receiver is based on a low-power Silicon Labs Happy Gecko microcontroller (MCU), specifically, the [EFM32HG310F64G](https://www.silabs.com/mcu/32-bit-microcontrollers/efm32-happy-gecko/device.efm32hg310f64g-qfn32), which has an Arm Cortex M0+ core, 64 KB flash memory, and 8 KB RAM.
The firmware is exclusively written in the C programming language.

* If you have a SnapperGPS receiver with an older version of the firmware and just want to update it, then you can just go to [the *Configure* page of the SnapperGPS app](https://snappergps.info/configure) and hit the *Update firmware* button.
* If you have a SnapperGPS receiver without firmware and want to install the most recent standard firmware, then you can download [the most recent binary](https://github.com/SnapperGPS/snappergps-firmware/releases) and go to the section on [Flashing](#flashing).
* If you want to modify the standard firmware, then keep reading.

### Table of contents

  * [Repository Structure](#repository-structure)
  * [Building](#building)
  * [Flashing](#flashing)
  * [Library](#library)
  * [USB Descriptor](#usb-descriptor)
  * [WebUSB Messages](#webusb-messages)
  * [Acknowledgements](#acknowledgements)

## Repository Structure

The firmware is split into two parts:

* A *shared_library* that aims to contain functions that are useful across different firmware version. This includes parts of the [CMSIS](https://developer.arm.com/tools-and-software/embedded/cmsis), the [EMLIB](https://docs.silabs.com/gecko-platform/latest/emlib/api), and the EMUSB libraries as well as custom functions for the timer, the analogue-to-digital converter (ADC), the flash, and the radio of a SnapperGPS receiver.
* A directory *firmware_versions*, which may contain different variants of the firmware that all build on the *shared-library*. As of writing, this repository just contains the general-purpose standard firmware. If you want to create your own firmware versions, then you can add additional sub-directories to *firmware_versions* with the same structure as the existing one.

## Building

To build the SnapperGPS firmware, the directory [firmware_versions/snapper/build](firmware_versions/snapper/build) contains a [make file](firmware_versions/snapper/build/Makefile)  and a [linker file](firmware_versions/snapper/build/snapper.ld).

Building requires the [Arm GNU Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain), which is a compiler toolchain for Arm based CPUs that is free to download and use (as of writing) and runs on GNU/Linux, Windows, and macOS.
To use it to build your own SnapperGPS firmware, proceed with the following steps:

* Download the toolchain for your OS from [here](https://developer.arm.com/downloads/-/gnu-rm). Select version 10.3 or an earlier version.
* Unpack the files to a directory of your choice.
* Make sure that you can run the `make` command on your system. Depending on your OS, you might have to set it up first.
* Open the [SnapperGPS make file](firmware_versions/snapper/build/Makefile) with an editor of your choice.
* Ensure that `TOOLCHAIN_PATH` points to the directory where you unpacked the Arm GNU Toolchain.
* Ensure that the `TOOLCHAIN_VERSION` coincidences with the version that you downloaded.
* Open a terminal.
* Change the directory to [firmware_versions/snapper/build](firmware_versions/snapper/build).
* Execute `make`.
* The firmware should build now and once the process is completed, `snapper.bin` will be available in the directory and can be flashed to a SnapperGPS receiver, see [next section](#flashing).

A few notes:

* If you want to change the name of the binary, open the make file and modify `FILENAME`.
* If you changed the directory of the shared library, open the make file and modify `SHARED`.
* If you are using a different microcontroller, then open the make file and modify `TARGET`. The default is `EFM32HG310F64`.
* If you want to change where the firmware is placed in the flash memory of the microcontroller or if you are using a microcontroller with a different flash memory size, then open the linker file and modify the `ORIGIN` and the `LENGTH` of the `FLASH`. The default reserves 16 KB at the beginning for the bootloader, leaving 48 KB for the SnapperGPS firmware.

## Flashing


Four ways of flashing the firmware in this directory, depending on which
firmware/bootloader is present on the device:

- [Flash the most recent SnapperGPS firmware to a device that runs SnapperGPS firmware already](#flash-the-most-recent-snappergps-firmware-to-a-device-that-runs-snappergps-firmware-already)
- [Flash custom firmware to a device that supports at least a subset of the SnapperGPS USB messages](#flash-custom-firmware-to-a-device-that-supports-at-least-a-subset-of-the-snappergps-usb-messages)
- [Flash custom firmware to a device that comes with Silicon Lab's USB bootloader](#flash-custom-firmware-to-a-device-that-comes-with-silicon-labs-usb-bootloader)
- [Flash custom firmware to a device that exposes Silicon Lab's Serial Wire Debug (SWD) interface](#flash-custom-firmware-to-a-device-that-exposes-silicon-labs-serial-wire-debug-swd-interface)

### Flash the most recent SnapperGPS firmware to a device that runs SnapperGPS firmware already

- Connect your SnapperGPS receiver via USB.
- Go to https://snappergps.info/configure.
- Pair your receiver, if necessary.
- Update the firmware using the "Update firmware" button.

### Flash custom firmware to a device that supports at least a subset of the SnapperGPS USB messages

- Connect your device via USB.
- Go to https://snappergps.info/flash.
- Pair your device, if necessary.
- Select the firmware binary on your host computer.
- Update the firmware using the "Update firmware" button.

### Flash custom firmware to a device that comes with Silicon Lab's USB bootloader

- Get the flash application from https://github.com/OpenAcousticDevices/EFM32-Flash.
- Open a terminal.
- Navigate to `\Flash\bin\your_OS`.
- Shorten the SWCLK and VDD pins of your board, 
- Connect your device via USB.
- Identify to which port it was assigned (should show as a serial device in Window's Device Manager).
- Execute `./flash -u COM42 "path_to_firmware\build\firmware.bin"` with the correct port and file path.

### Flash custom firmware to a device that exposes Silicon Labs' Serial Wire Debug (SWD) interface

- Below a description how to achieve this using [an EFM32 Giant Gecko Starter Kit](https://www.silabs.com/development-tools/mcu/32-bit/efm32gg-starter-kit) and [the Silicon Labs Simplicity Commander](https://community.silabs.com/s/article/simplicity-commander). Although, any programmer with an SWD interface can be employed.
- Connect the EFM32 Giant Gecko Starter Kit via its micro-USB port to your computer.
- Use jump wires (and optionally a male 0.1-inch pin header) to connect the SnapperGPS board to the vertical debug header of the Starter Kit. Connect *RESET* to *#RESET*, *SWCLK* to *SWCLK*, *SWDIO* to *SWDIO*, *GND* to *GND*, and *VDD* to *VTARGET*. Below the pinout of the Starter Kit.
```
             ┌─────┐
  VTARGET  1 │ □ □ │  2 NC
    #TRST  3 │ □ □ │  4 GND
      TDI  5 │ □ □ │  6 GND
TMS/SWDIO  7 │ □ □ │  8 GND
TCK/SWCLK  9 └┐□ □ │ 10 GND
     RTCK 11 ┌┘□ □ │ 12 GND
  TDO/SWO 13 │ □ □ │ 14 GND
   #RESET 15 │ □ □ │ 16 GND
       PD 17 │ □ □ │ 18 Cable Detect
       PD 19 │ □ □ │ 20 GND
             └─────┘
```
- Power the SnapperGPS board from a voltage source, e.g., a lithium-ion polymer battery. You could even use the Starter Kit for this, it provides 3.3 V and 5 V outputs.
- Run the Simplicity Commander.
- Choose the connected Starter Kit in the first field of the top row. Leave the debug interface as *SWD* and the rate as *8000 kHz*.
- Choose *EFM32HG310F64* as device.
- Select the *Flash* tab.
- Select your firmware (the `.bin` file in your `build` directory) as binary file for flashing.
- Set the flash start address to the `ORIGIN` value from your `make` file \[double check\].
- Click on *Flash*. The firmware should be uploaded now.

### Flash the Silicon Labs USB bootloader

- All Happy Gecko MCUs from revision B onward ship with [a USB bootloader](https://www.silabs.com/documents/public/application-notes/an0042-efm32-usb-uart-bootloader.pdf).
E.g., it allows to shorten *VDD* and *SWCLK*, then to connect via USB, and then to flash the device via the AudioMoth Flash App. However, if the bootloader was erased accidently or a revision A MCU is used, then you can flash it using the SWD interface.
- Download [bootloader binaries from Silicon Labs](https://www.silabs.com/documents/public/example-code/an0042-efm32-usb-uart-bootloader.zip).
- Unpack the `.zip` file.
- Proceed with the same steps as in the section above, however:
- Set the flash start address to `0x00`.
- Use the `.hex` file with the most recent version number from the `binaries` directory in the downloaded `.zip` file.

## Library

A brief description of the functions in the [*shared library*](shared_library):

### analogToDigitalConverter

*Functions to make various measurements with an analog-to-digital converter (ADC) of the Happy Gecko MCU:*

```C
void AnalogToDigitalConverter_enable()
```

> Enable the ADC, which is required before making any measurements.

```C
void AnalogToDigitalConverter_disable()
```

> Disable the ADC after making measurements.

```C
void AnalogToDigitalConverter_enableBatteryMeasurement(pinPullValue_t enablePullValue)
```

> Enable the battery voltage measurement circuit, which needs to be done separatly.
> Some versions of the SnapperGPS receiver require the pin for this to be pulled low, some require it to be pulled high.
> Pass `PULL_LOW` or `PULL_HIGH`, respectively.
> The SnapperGPS receiver V1.0.0 works with `PULL_HIGH`.

```C
void AnalogToDigitalConverter_disableBatteryMeasurement()
```

> Disable the battery voltage measurement circuit after making a measurement (to save energy).

```C
uint32_t AnalogToDigitalConverter_measureVDD()
```

> Measure the supply voltage in centi-volts.
> The ADC needs to be enabled.

```C
int32_t AnalogToDigitalConverter_measureTemperature()
```

> Measure the MCU temperature in centi-degrees Celsius.
> The ADC needs to be enabled.

```C
uint32_t AnalogToDigitalConverter_measureBatteryVoltage()
```

> Measure the battery voltage in centi-volts.
> The ADC and the battery voltage measurement ciruit need to be enabled.

### flash

*To understand the functions for the external flash memory (W25N512GV), it might be useful to have a look at chapter 8 of [its datasheet](https://www.winbond.com/resource-files/W25N512GV%20Rev%20F%20042621.pdf) since many functions map to its instructions.*

*Macros defined for the flash memory:*

```C
FLASH_USART
```

> Which USART of the MCU to use as serial interface for the flash memory chip (default: `USART0`).

```C
FLASH_PAGE_LENGTH
```

> Number of bytes in one page of the flash memory (default: `2048`).

```C
FLASH_PAGES_PER_BLOCK
```

> Number of pages per block in the flash memory (default: `64`).

```C
FLASH_NUM_BLOCKS
```

> Number of blocks in the flash memory (default: `512`).

```C
FLASH_NUM_PAGES
```

> Number of pages in the external flash memory (default: `(FLASH_NUM_BLOCKS * FLASH_PAGES_PER_BLOCK)`).

*Some useful functions defined for the flash memory:*

```C
void Flash_powerOn()
```

> Power on the flash memory before using it.

```C
void Flash_powerOff()
```

> Power of the flash memory when it is not in use to save energy.

```C
void Flash_enableInterface()
```

> Enable the serial interface of the flash.
> It is best to wait ~1 ms after powering it on (`Flash_powerOn`) before enabling its interface, e.g., with `Timer_delayMilliseconds(1)`.

```C
void Flash_disableInterface()
```

> Disable the serial interface of the flash before powering it off with `Flash_powerOff`.

```C
void Flash_reset()
```

```C
void Flash_erase()
```

> Erase the whole flash memory.
> Call `Flash_removeProtection` first and wait while `Flash_isBusy` before erasing.

```C
void Flash_blockErase(uint32_t address)
```

> Erase the block with the given start address.
> Call `Flash_removeProtection` first and wait while `Flash_isBusy` before doing so.

```C
bool Flash_isBusy()
```

> Check if the busy flag of the flash memory is set.

```C
void Flash_readID(uint32_t *id)
```

```C
void Flash_readStatus(uint8_t *status)
```

```C
void Flash_enterDeepPowerDownMode()
```

```C
void Flash_leaveDeepPowerDownMode()
```

```C
void Flash_readProtectionRegister(uint8_t *status)
```

```C
void Flash_removeProtection()
```

> Makes blocks writeable.
> Call after checking that flash is not busy with `Flash_isBusy`.
> Call before erasing and writing.

```C
void Flash_readConfigurationRegister(uint8_t *status)
```

```C
void Flash_writeConfigurationRegister(uint8_t *status)
```

```C
FlashResult_t Flash_readBytes(uint32_t address, uint8_t *dest, uint32_t length)
```

> Read a certain number of bytes (`length`) starting at a flash memory `address` into a destination array at `dest`.
> The return value will either be `FLASH_SUCCESS` or `FLASH_ADDRESS_INVALID`.
> Call after checking that flash is not busy with `Flash_isBusy`.

```C
FlashResult_t Flash_readPage(uint32_t address, uint8_t *dest)
```

> Read a whole page starting at a flash memory `address` into a destination array at `dest`.
> The return value will either be `FLASH_SUCCESS` or `FLASH_ADDRESS_INVALID`.
> Call after checking that flash is not busy with `Flash_isBusy`.

```C
FlashResult_t Flash_writePage(uint32_t address, uint8_t *src)
```

> Write a whole page starting at a flash memory `address` into a destination array at `dest`.
> The return value will either be `FLASH_SUCCESS` or `FLASH_ADDRESS_INVALID`.
> Call after `Flash_removeProtection` and after checking that flash is not busy with `Flash_isBusy`.

```C
FlashResult_t Flash_enableContinuousRead(uint32_t address)
```

> Enable continues reading from a certain flash start `address`.
> Consult the datasheet for details on this mode.

```C
uint8_t Flash_readContinuousByte()
```

> Read the next unread byte in continuous mode.

```C
void Flash_disableContinuousRead()
```

> Disable continuous reading.

```C
void Flash_init()
```

> Check if flash memory is 1 GBit (W25N01GV) or 512 MBit (W25N512GV) and set parameters for following function calls accordingly.
> Default is 512 MBit (W25N512GV).
> If 1 GBit (W25N01GV) is used, then this function must be called once in your main function after `Flash_powerOn()` and `Flash_enableInterface()` before any other flash function is called.

### radio

*Functions for the receiver IC (the radio):*

```C
void Radio_powerOn()
```

> Power on the radio.

```C
void Radio_powerOff()
```

> Power off the radio.

```C
void Radio_enableHFXOInput()
```

> Enable the 16 MHz oscillator.
> Call after powering on the radio.
> Then, wait until its frequency has stabilised, e.g., with `Timer_delayMilliseconds(10)`.

```C
void Radio_disableHFXOInput()
```

> Disable the 16 MHz oscillator.
> Call before powering off the radio.

```C
void Radio_sampleByte(uint8_t *byte)
```

> Capture a single byte from the GNSS signal.
> The eight bits represent eight consecutive amplitude values.
> The bitorder is little-endian.

```C
void Radio_captureSnapshot(uint8_t *dest, uint32_t length)
```

> Capture a GNSS signal snapshot with `length` bytes into memory at `dest`.
> Each byte represents eight consecutive amplitude values.
> The bitorder is little-endian.

### timer

*Functions for a timer of the MCU:*

```C
void Timer_enable()
```

> Enable the timer.

```C
void Timer_disable()
```

> Disable the timer.

```C
void Timer_delayMilliseconds(uint32_t milliseconds)
```

> Pause execution some `milliseconds`.

```C
void Timer_delayMicroseconds(uint32_t microseconds)
```

> Pause execution some `microseconds`.

## USB Descriptor

Find the USB descriptor in [firmware_versions/snapper/inc/usbdescriptors.h](firmware_versions/snapper/inc/usbdescriptors.h).

The `URL_Descriptor` contains three bytes followed by the URL to which the user is re-directed when the SnapperGPS receiver is plugged in.
Note that the first byte encodes the length of the whole URL descriptor and, therefore, must be adjusted when the length of the URL is changed.

There is also a section with *String Descriptors*, which contain information that is communicated to the host operating system once a SnapperGPS receiver is connected via USB.
This includes manufacturer, product description, and serial number.
E.g., the Device Manager on Microsoft Windows and the System Information / System Report on macOS present this information ot the user.

In general, the firmware is configured to work with WebUSB as a bulk endpoint, see the configuration descriptor `configDesc`.

Macros relevant for the USB configuration on board of the SnapperGPS receiver are defined in [firmware_versions/snapper/inc/usbconfig.h](firmware_versions/snapper/inc/usbconfig.h).
This includes the endpoints (in/out) and the clock used for USB.

## WebUSB Messages

The messages are split into two sets, one which contains messages that are
relevant for generic devices and one set that contains messages that are only
relevant to SnapperGPS.

The first byte of a WebUSB message defines the message type.
Generic messages are identified by hexadecimal numbers of the form `0x0N` with
`N` being a hexadecimal digit.
device-specific messages have numbers of the form `0x8N`.
Almost all messages trigger an echo of this byte.

If not stated otherwise, all incoming messages are 64 bytes long and all
outgoing messages have a length of 128 bytes.

### Generic messages

There are the following generic messages.
Together with the device-specific messages, they can form an enumeration:

```C
typedef enum {
  SET_TIME_MESSAGE = 0x01,
  GET_STATUS_MESSAGE = 0x02,
  SET_FIRMWARE_INIT_MESSAGE = 0x03,
  SET_FIRMWARE_PAGE_MESSAGE = 0x04,
  GET_FIRMWARE_CRC_MESSAGE = 0x05,
  SET_FIRMWARE_FLASH_MESSAGE = 0x06
  // Device-specific messages to follow
} usbMessageType_t;
```

The last four messages are for loading new firmware.
There is a small [website](https://snappergps.info/flash) that allows
to flash any device that supports the generic messages.
It first sends `SET_FIRMWARE_INIT_MESSAGE`, followed by the correct number of
`SET_FIRMWARE_PAGE_MESSAGE`, and finally `GET_FIRMWARE_CRC_MESSAGE`.
If the CRC passes, then it completes the update with
`SET_FIRMWARE_FLASH_MESSAGE`.

Below, you can first find the definitions and descriptions for the incoming
messages, in struct format, and then for the outgoing messages.
The field `msgType` refers to the respective hexadecimal number above.

**`SET_TIME_MESSAGE`**

Set the time of the clock of the device. Echo message type.

```C
typedef struct {
  usbMessageType_t msgType
  uint32_t time;                            // Unix timestamp
  uint32_t ticks;                           // Sub-second ticks [0 to 1023]
} usbMessageSetTimeIn_t;
```

```C
typedef struct {
  usbMessageType_t msgType
} usbMessageSetTimeOut_t;
```

**`GET_STATUS_MESSAGE`**

Get general information from the device (time, battery voltage, ID, firmware).
Echo message type.

```C
typedef struct {
  usbMessageType_t msgType
} usbMessageGetStatusIn_t;
```

```C
typedef struct {
  usbMessageType_t msgType
  uint32_t time;                      // Unix timestamp (device clock)
  uint32_t ticks;                     // Ticks in second [0 to 1023]
  uint32_t batteryVoltage;            // Battery voltage in hundreds of volts
  uint64_t deviceID;
  uint8_t firmwareDescription[32];
  uint8_t firmwareVersion[3];
  uint32_t firmwareSize;              // Length of firmware in bytes that is expected during a firmware update (or 0 if firmware update is not supported)
  uint16_t firmwareChunkSize;         // Length of individual firmware chunks in bytes in which the firmware is sent during an update  (or 0 if firmware update is not supported)
  // Everything that follows is device-specific information
} usbMessageGetStatusOut_t;
```

**`SET_FIRMWARE_INIT_MESSAGE`**

Announce that firmware will be sent next, in chunks, using
`SET_FIRMWARE_MESSAGE`.
E.g., a SnapperGPS receiver expects 48 KB of firmware to be sent subsequently.
This is equal to the `firmwareSize` field from the `usbMessageGetStatusOut_t`
struct.
Erase flash block reserved for firmware.
Firmware can be written to last 128 KB block of external flash memory for
SnapperGPS.
Echo message type.

```C
typedef struct {
  usbMessageType_t msgType
} usbMessageSetFirmwareInitIn_t; 
```

```C
typedef struct {
  usbMessageType_t msgType 
} usbMessageSetFirmwareInitOut_t; 
```

**`SET_FIRMWARE_PAGE_MESSAGE`**

Just 2048 bytes of firmware, **no** message type as preamble.
The 2-KB chunk corresponds to SnapperGPS' flash page size.
Another device could use another chunk size.
This is equal to the `firmwareChunkSize` field of the `usbMessageGetStatusOut_t`
struct.
Return message type.

```C
typedef struct {
  uint8_t firmwarePage[FIRMWARE_CHUNK_SIZE] 
} usbMessageSetFirmwarePageIn_t;
```

```C
typedef struct {
  usbMessageType_t msgType 
} usbMessageSetFirmwareInitOut_t;
```

**`GET_FIRMWARE_CRC_MESSAGE`**

Echo message type.
Calculate check value for CRC based on firmware in external flash (48 KB for
SnapperGPS).

```C
typedef struct {
  usbMessageType_t msgType,
} usbMessageSetRecordIn_t;
```

```C
typedef struct {
  usbMessageType_t msgType, 
  uint16_t crc                      // Calculated check value for CRC
} usbMessageSetFirmwareCrcIn_t;
```

**`SET_FIRMWARE_FLASH_MESSAGE`**

Echo message type.
Call SRAM function to write firmware to internal flash.
Reset to start the new firmware.

```C
typedef struct {
  usbMessageType_t msgType 
} usbMessageSetFirmwareFlashIn_t;
```

```C
typedef struct {
  usbMessageType_t msgType
} usbMessageSetFirmwareFlashOut_t;
```

### SnapperGPS-specific messages

There are the following SnapperGPS-specific messages:

```C
typedef enum {
  // Generic messages to precede
  // SnapperGPS-specific messages:
  GET_METADATA_MESSAGE = 0x81,
  SET_RECORD_MESSAGE = 0x82,
  SET_SHUTDOWN_MESSAGE = 0x83,
  GET_SNAPSHOT_MESSAGE = 0x84
} usbMessageType_t;
```

In addition, `GET_STATUS_MESSAGE` doubles as a generic and a device-specific
message.

**`GET_STATUS_MESSAGE`**

Get general information from the device.
Echo message type.
(Listed here again because it doubles as generic and device-specific message).

```C
typedef struct {
  usbMessageType_t msgType
} usbMessageGetStatusIn_t;
```

```C
typedef struct {
    // Generic information as defined above (first 62 bytes)
    // Device-specific information
    uint8_t state;                                // Device state
    uint16_t snapshotCount;                       // Number of stored snapshots
} usbMessageGetStatusOut_t;
```

The following SnapperGPS states are defined:

```C
typedef enum {
  STATE_WILL_SHUTDOWN,   // Device will shutdown after USB is unplugged
  STATE_WILL_RECORD,     // Device will record snapshots after USB is unplugged
  STATE_ERASING          // Device is erasing the external flash
} deviceState_t;
```

**`GET_METADATA_MESSAGE`**

Get meta data of next unread snapshot (validity, time, temperature, battery
voltage) or info that all snapshots have been read.
The `valid` field indicates whether the subsequent data is valid or not.
If data is valid, pointer has been set to next potential meta data.
If data is invalid, pointer has been reset to start of data in flash.
If valid, raw snapshot can be obtained with subsequent `GET_SNAPSHOT_MESSAGE`.
Meta data of next snapshot can be obtained with subsequent
`GET_METADATA_MESSAGE`.
Echo message type.

```C
typedef struct {
  usbMessageType_t msgType
} usbMessageGetMetadataIn_t;
```

```C
typedef struct {
  usbMessageType_t msgType,
  bool valid,                        // Whether the subsequent data is valid  
  uint32_t time,                     // Unix timestamp (snapshot capture)
  uint32_t ticks,                    // Clock ticks (snapshot capture) [0-1023]
  uint32_t temperature,              // Temp. in tenth of degrees, 1024 offset
  uint32_t batteryVoltage            // Battery voltage in hundreds of volts
} usbMessageGetMetadataOut_t;
```

**`SET_RECORD_MESSAGE`**

Echo message type.
Erase external flash memory, go to state `STATE_ERASING` while doing so.
Go to state `STATE_WILL_RECORD`.
After un-plugging, start recording snapshots in defined intervals after start
time.
After end time, stop recording snapshots and go to state `STATE_WILL_SHUTDOWN`.

```C
typedef struct {
  usbMessageType_t msgType,
  uint32_t measurementInterval,          // Time between 2 snapshots in seconds
  uint32_t startTime,                    // Unix timestamp of first snapshot
  uint32_t endTime                       // Unix timestamp of last snapshot
} usbMessageSetRecordIn_t;
```

```C
typedef struct {
  usbMessageType_t msgType
} usbMessageSetRecordOut_t;
```

**`SET_SHUTDOWN_MESSAGE`**

Echo message type.
Disable RTC interrupts.
Go to state `STATE_WILL_SHUTDOWN`.
After un-plugging, device will shut down. 

```C
typedef struct {
  usbMessageType_t msgType
} usbMessageSetShutdownIn_t;
```

```C
typedef struct {
  usbMessageType_t msgType
} usbMessageSetShutdownOut_t;
```

**`GET_SNAPSHOT_MESSAGE`**

Get binary raw data of next unread snapshot.
Return just all bytes of the snapshot.
Do **not** echo message type.

```C
typedef struct {
  usbMessageType_t msgType
} usbMessageGetSnapshotIn_t; 
```

```C
typedef struct {
  uint8_t snapshot[6144]
} usbMessageGetSnapshotOut_t;
```

## Acknowledgements

This SnapperGPS firmware was developed by
[Jonas Beuchert](https://users.ox.ac.uk/~kell5462/) and
[Alex Rogers](https://www.cs.ox.ac.uk/people/alex.rogers/)
in the Department of Computer Science
of the University of Oxford.

Jonas Beuchert is
funded by the EPSRC Centre for Doctoral Training in
Autonomous Intelligent Machines and Systems
(DFT00350-DF03.01) and develops
SnapperGPS as part of his doctoral studies.
The implementation of SnapperGPS 
was co-funded by an EPSRC IAA Technology Fund
(D4D00010-BL14).

##

This documentation is licensed under a
[Creative Commons Attribution 4.0 International License][cc-by].

[![CC BY 4.0][cc-by-image]][cc-by]

[cc-by]: http://creativecommons.org/licenses/by/4.0/
[cc-by-image]: https://i.creativecommons.org/l/by/4.0/88x31.png

