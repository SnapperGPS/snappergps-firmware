/****************************************************************************
 * usbdescriptors.h
 * SnapperGPS
 * November 2020
 *****************************************************************************/

#ifndef __USBDESCRIPTORS_H
#define __USBDESCRIPTORS_H

#define USB_EP0_SIZE        64
// Increase to 512?
#define USB_MAX_EP_SIZE     64

/* Binary device Object Store (BOS) Descriptor */

SL_ALIGN(4)
static const char BOS_Descriptor[] __attribute__ ((aligned(4))) = {

    /* Binary device Object Store descriptor */

    0x05,                                                   /* bLength                                  */
    0x0F,                                                   /* bDescriptorType                          */
    0x39,                                                   /* wTotalLength (LSB)                       */
    0x00,                                                   /* wTotalLength (MSB)                       */
    0x02,                                                   /* bNumDeviceCaps                           */

    /* WebUSB platform capability descriptor */

    0x18,                                                   /* bLength                                  */
    0x10,                                                   /* bDescriptorType                          */
    0x05,                                                   /* bDevCapabilityType                       */
    0x00,                                                   /* bReserved                                */
    0x38, 0xB6, 0x08, 0x34, 0xA9, 0x09,                     /* PlatformCapabilityUUID                   */
    0xA0, 0x47, 0x8B, 0xFD, 0xA0, 0x76,
    0x88, 0x15, 0xB6, 0x65,
    0x00,                                                   /* bcdVersion (LSB)                         */
    0x01,                                                   /* bcdVersion (MSB)                         */
    0x01,                                                   /* bVendorCode                              */
    0x01,                                                   /* iLandingPage                             */

    /* Microsoft OS 2.0 platform capability descriptor */

    0x1C,                                                   /* bLength                                  */
    0x10,                                                   /* bDescriptorType                          */
    0x05,                                                   /* bDevCapabilityType                       */
    0x00,                                                   /* bReserved                                */
    0xDF, 0x60, 0xDD, 0xD8, 0x89,                           /* PlatformCapabilityUUID                   */
    0x45, 0xC7, 0x4C, 0x9C, 0xD2,
    0x65, 0x9D, 0x9E, 0x64, 0x8A,
    0x9F,
    0x00,                                                   /* dwWindowsVersion (LSB)                   */
    0x00,                                                   /*         ...                              */
    0x03,                                                   /*         ...                              */
    0x06,                                                   /* dwWindowsVersion (MSB)                   */
    0x1e,  // 0xB2                                          /* wMSOSDescriptorSetTotalLength (LSB)      */
    0x00,                                                   /* wMSOSDescriptorSetTotalLength (MSB)      */
    0x02,                                                   /* bMS_VendorCode                           */
    0x00,                                                   /* bAltEnumCode                             */

};

/* URL Descriptor */

SL_ALIGN(4)
static const char URL_Descriptor[] __attribute__ ((aligned(4))) = {
    0x12,                                                   /* bLength                                  */
    0x03,                                                   /* bDescriptorType                          */
    0x01,                                                   /* bScheme                                  */
    's', 'n', 'a', 'p', 'p', 'e', 'r',                      /* URL                                      */
    'g', 'p', 's', '.', 'i', 'n', 'f', 
    'o',
};

/* Microsoft OS 2.0 Descriptor */

SL_ALIGN(4)
static const char MICROSOFT_Descriptor[] __attribute__ ((aligned(4))) = {

    /* Microsoft OS 2.0 descriptor set header */

    0x0A,                                                   /* wLength (LSB)                            */
    0x00,                                                   /* wLength (MSB)                            */
    0x00,                                                   /* wDescriptorType (MSB)                    */
    0x00,                                                   /* wDescriptorType (MSB)                    */
    0x00,                                                   /* dwWindowsVersion (LSB)                   */
    0x00,                                                   /*         ...                              */
    0x03,                                                   /*         ...                              */
    0x06,                                                   /* dwWindowsVersion (MSB)                   */
    0x1E,  // 0xB2                                          /* wTotalLength (LSB)                       */
    0x00,                                                   /* wTotalLength (MSB)                       */

    /* Microsoft OS 2.0 configuration subset header */

    // 0x08,                                                   /* wLength (LSB)                            */
    // 0x00,                                                   /* wLength (MSB)                            */
    // 0x01,                                                   /* wDescriptorType (LSB)                    */
    // 0x00,                                                   /* wDescriptorType (MSB)                    */
    // 0x00,                                                   /* bConfigurationValue                      */
    // 0x00,                                                   /* bReserved                                */
    // 0xA8,                                                   /* wTotalLength (LSB)                       */
    // 0x00,                                                   /* wTotalLength (MSB)                       */

    /* Microsoft OS 2.0 function subset header */

    // 0x08,                                                   /* wLength (LSB)                            */
    // 0x00,                                                   /* wLength (MSB)                            */
    // 0x02,                                                   /* wDescriptorType (LSB)                    */
    // 0x00,                                                   /* wDescriptorType (MSB)                    */
    // 0x01,                                                   /* bFirstInterface                          */
    // 0x00,                                                   /* bReserved                                */
    // 0xA0,                                                   /* wSubsetLength (LSB)                      */
    // 0x00,                                                   /* wSubsetLength (MSB)                      */

    /* Microsoft OS 2.0 compatible ID descriptor */

    0x14,                                                   /* wLength (LSB)                            */
    0x00,                                                   /* wLength (MSB)                            */
    0x03,                                                   /* wDescriptorType (LSB)                    */
    0x00,                                                   /* wDescriptorType (MSB)                    */
    'W', 'I', 'N', 'U',                                     /* CompatibileID                            */
    'S', 'B', '\0', '\0',
    '\0', '\0', '\0', '\0',                                 /* SubCompatibleID                          */
    '\0', '\0', '\0', '\0',

    /* Microsoft OS 2.0 registry property descriptor */

    // 0x84,                                                   /* wLength (LSB)                            */
    // 0x00,                                                   /* wLength (MSB)                            */
    // 0x04,                                                   /* wDescriptorType (LSB)                    */
    // 0x00,                                                   /* wDescriptorType (MSB)                    */
    // 0x07,                                                   /* wPropertyDataType (LSB)                  */
    // 0x00,                                                   /* wPropertyDataType (MSB)                  */
    // 0x2A,                                                   /* wPropertyNameLength (LSB)                */
    // 0x00,                                                   /* wPropertyNameLength (MSB)                */
    // 'D', 0x00, 'e', 0x00, 'v', 0x00,                        /* PropertyName (UT16-LE)                   */
    // 'i', 0x00, 'c', 0x00, 'e', 0x00,
    // 'I', 0x00, 'n', 0x00, 't', 0x00,
    // 'e', 0x00, 'r', 0x00, 'f', 0x00,
    // 'a', 0x00, 'c', 0x00, 'e', 0x00,
    // 'G', 0x00, 'U', 0x00, 'I', 0x00,
    // 'D', 0x00, 's', 0x00, '\0', 0x00,
    // 0x50,                                                   /* wPropertyDataLength (LSB)                */
    // 0x00,                                                   /* wPropertyDataLength (MSB)                */
    // '{', 0x00, '0', 0x00, 'a', 0x00,                        /* PropertyData ((UT16-LE)                  */
    // 'a', 0x00, '6', 0x00, '9', 0x00,
    // 'e', 0x00, 'c', 0x00, 'f', 0x00,
    // '-', 0x00, 'a', 0x00, 'e', 0x00,
    // 'b', 0x00, '9', 0x00, '-', 0x00,
    // '4', 0x00, '0', 0x00, '3', 0x00,
    // 'a', 0x00, '-', 0x00, 'b', 0x00,
    // '6', 0x00, '8', 0x00, '2', 0x00,
    // '-', 0x00, '3', 0x00, '4', 0x00,
    // '9', 0x00, 'd', 0x00, '5', 0x00,
    // '0', 0x00, '6', 0x00, 'e', 0x00,
    // '2', 0x00, '0', 0x00, '0', 0x00,
    // 'd', 0x00, '}', 0x00, '\0', 0x00,
    // '\0', 0x00

};

/* Device Descriptor */

SL_ALIGN(4)
static const USB_DeviceDescriptor_TypeDef deviceDesc __attribute__ ((aligned(4)))= {

    .bLength            = USB_DEVICE_DESCSIZE,              /* Size of the Descriptor in Bytes          */
    .bDescriptorType    = USB_DEVICE_DESCRIPTOR,            /* Device Descriptor type                   */
    .bcdUSB             = 0x0210,                           /* USB 2.1 compliant                        */
    .bDeviceClass       = 0x00,                             /* Vendor unique device                     */
    .bDeviceSubClass    = 0x00,                             /* Ignored for vendor unique device         */
    .bDeviceProtocol    = 0x00,                             /* Ignored for vendor unique device         */
    .bMaxPacketSize0    = USB_EP0_SIZE,                     /* Max packet size for EP0                  */
    .idVendor           = 0x10C4,                           /* VID                                      */
    .idProduct          = 0x0002,                           /* PID                                      */
    .bcdDevice          = 0x0000,                           /* Device Release number                    */
    .iManufacturer      = 0x01,                             /* Index of Manufacturer String Descriptor  */
    .iProduct           = 0x02,                             /* Index of Product String Descriptor       */
    .iSerialNumber      = 0x03,                             /* Index of Serial Number String Descriptor */
    .bNumConfigurations = 0x01                              /* Number of Possible Configurations        */
};

/* Configuration Descriptor */

SL_ALIGN(4)
static const uint8_t configDesc[] __attribute__ ((aligned(4)))= {

    /* Configuration descriptor */

    USB_CONFIG_DESCSIZE,                                    /* bLength                                  */
    USB_CONFIG_DESCRIPTOR,                                  /* bDescriptorType                          */

    USB_CONFIG_DESCSIZE +                                   /* wTotalLength (LSB)                       */
    USB_INTERFACE_DESCSIZE +
    (USB_ENDPOINT_DESCSIZE * NUM_EP_USED),

    (USB_CONFIG_DESCSIZE +                                  /* wTotalLength (MSB)                       */
    USB_INTERFACE_DESCSIZE +
    (USB_ENDPOINT_DESCSIZE * NUM_EP_USED))>>8,

    0x01,                                                   /* bNumInterfaces                           */
    0x01,                                                   /* bConfigurationValue                      */
    0x00,                                                   /* iConfiguration                           */
    CONFIG_DESC_BM_RESERVED_D7 |                            /* bmAttrib                                 */
    CONFIG_DESC_BM_SELFPOWERED |
    CONFIG_DESC_BM_REMOTEWAKEUP,
    CONFIG_DESC_MAXPOWER_mA(100),                           /* bMaxPower: 100 mA                        */

    /* Interface descriptor */

    USB_INTERFACE_DESCSIZE,                                 /* bLength                                  */
    USB_INTERFACE_DESCRIPTOR,                               /* bDescriptorType                          */
    0x00,                                                   /* bInterfaceNumber                         */
    0x00,                                                   /* bAlternateSetting                        */
    NUM_EP_USED,                                            /* bNumEndpoints                            */
    0xFF,                                                   /* bInterfaceClass                          */
    0x00,                                                   /* bInterfaceSubClass                       */
    0x00,                                                   /* bInterfaceProtocol                       */
    0x00,                                                   /* iInterface                               */

    /* Interrupt End-point Descriptor (OUT, host to device) */

    USB_ENDPOINT_DESCSIZE,                                  /* bLength                                  */
    USB_ENDPOINT_DESCRIPTOR,                                /* bDescriptorType                          */
    WEBUSB_EP_OUT,                                          /* bEndpointAddress                         */
    // Change from "USB_ETYPE_INT" to "USB_ETYPE_BULK"?
    USB_EPTYPE_BULK,                                        /* bmAttributes                             */
    USB_MAX_EP_SIZE,                                        /* wMaxPacketSize (LSB)                     */
    0x00,                                                   /* wMaxPacketSize (MSB)                     */
    // Change from "0x01" to "0x00"? (Ignored for bulk transfer.)
    0x00,                                                   /* bInterval                                */

    /* Interrupt End-point Descriptor (IN, device to host) */

    USB_ENDPOINT_DESCSIZE,                                  /* bLength                                  */
    USB_ENDPOINT_DESCRIPTOR,                                /* bDescriptorType                          */
    WEBUSB_EP_IN,                                           /* bEndpointAddress                         */
    // Change from "USB_ETYPE_INT" to "USB_ETYPE_BULK"?
    USB_EPTYPE_BULK,                                        /* bmAttributes                             */
    USB_MAX_EP_SIZE,                                        /* wMaxPacketSize (LSB)                     */
    0x00,                                                   /* wMaxPacketSize (MSB)                     */
    // Change from "0x01" to "0x00"? (Ignored for bulk transfer.)
    0x00                                                    /* bInterval                                */

};

/* String Descriptors */

STATIC_CONST_STRING_DESC_LANGID(langID, 0x04, 0x09);

STATIC_CONST_STRING_DESC(iManufacturer, 's', 'n', 'a', 'p', 'p', 'e', 'r', 'g', 'p', 's', '.', 'i', 'n', 'f', 'o');

STATIC_CONST_STRING_DESC(iProduct, 'S','n','a','p','p','e','r', 'G', 'P', 'S');

STATIC_CONST_STRING_DESC(iSerialNumber, '0', '1','0','0');

/* End-point buffer sizes. Uses 1 for Control/Interrupt end-points and 2 for Bulk end-points */

static const uint8_t bufferingMultiplier[NUM_EP_USED + 1] = {
    1,  /* Control */
    // Change to "2" (bulk)?
    1,  /* Interrupt */
    // Change to "2" (bulk)?
    1   /* Interrupt */
};

/* Define string array */

static const void * const strings[] = {
    &langID,
    &iManufacturer,
    &iProduct,
    &iSerialNumber
};

/* Define call backs that are called by the USB stack on different events */

static const USBD_Callbacks_TypeDef callbacks = {
    .usbReset        = NULL,              /* Called whenever USB reset signalling is detected on the USB port.                                           */
    .usbStateChange  = stateChange,       /* Called whenever the device change state.                                                                    */
    .setupCmd        = setupCmd,          /* Called on each setup request received from host.                                                            */
    .isSelfPowered   = NULL,              /* Called whenever the device stack needs to query if the device is currently self- or bus-powered.            */
    .sofInt          = NULL               /* Called at each SOF (Start of Frame) interrupt. If NULL, the device stack will not enable the SOF interrupt. */
};

/* Structure passed to USBD_Init() in order to initialise the USB stack */

static const USBD_Init_TypeDef usbInitStruct = {
    .deviceDescriptor    = &deviceDesc,
    .configDescriptor    = configDesc,
    .stringDescriptors   = strings,
    .numberOfStrings     = sizeof(strings)/sizeof(void*),
    .callbacks           = &callbacks,
    .bufferingMultiplier = bufferingMultiplier,
    .reserved            = 0
};

#endif /* __USBDESCRIPTORS_H */
