/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "bmc.h"

#if USE_PACKETUSB

#include "packet_usb.h"

/*
 * Useful sites:
 *  http://www.usbmadesimple.co.uk
 *  http://www.beyondlogic.org/usbnutshell/usb1.shtml
 *  https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt
 *  This may be worth looking into: https://github.com/pidcodes/pidcodes.github.com
 */


/**
 * @brief   IN EP1 state.
 */
static USBInEndpointState ep1instate;

/**
 * @brief   OUT EP2 state.
 */
static USBOutEndpointState ep2outstate;

#if BMC_USE_USB_EXTRA_ENDPOINTS
/**
 * @brief   IN EP3 state.
 */
static USBInEndpointState ep3instate;

/**
 * @brief   OUT EP4 state.
 */
static USBOutEndpointState ep4outstate;
#endif

/*
 * USB Device Descriptor.
 */
static const uint8_t vcom_device_descriptor_data[18] = {
  USB_DESC_DEVICE       (0x0200,        /* bcdUSB (2.0).                    */
                         0xFF,          /* bDeviceClass (CDC).              */
                         0x00,          /* bDeviceSubClass.                 */
                         0x00,          /* bDeviceProtocol.                 */
                         0x40,          /* bMaxPacketSize.                  */
                         0x27d8,        /* idVendor .                   */
                         0x16c0,        /* idProduct.                       */
                         0x0001,        /* bcdDevice.                       */
                         1,             /* iManufacturer.                   */
                         2,             /* iProduct.                        */
                         3,             /* iSerialNumber.                   */
                         1)             /* bNumConfigurations.              */
};

/*
 * Device Descriptor wrapper.
 */
static const USBDescriptor vcom_device_descriptor = {
  sizeof vcom_device_descriptor_data,
  vcom_device_descriptor_data
};

#if BMC_USE_USB_EXTRA_ENDPOINTS
#define USBCONFIGSIZE 46
#else
#define USBCONFIGSIZE 32
#endif

/* Configuration Descriptor tree for a CDC.*/
static const uint8_t vcom_configuration_descriptor_data[USBCONFIGSIZE] = {
  /* Configuration Descriptor. 9 bytes */
  USB_DESC_CONFIGURATION(USBCONFIGSIZE,            /* wTotalLength.                    */
                         0x01,          /* bNumInterfaces.                  */
                         0x01,          /* bConfigurationValue.             */
                         0,             /* iConfiguration.                  */
                         0x80,          /* bmAttributes.                    */
                         50),           /* bMaxPower (100mA).               */
  /* Interface Descriptor. 9 bytes*/
  USB_DESC_INTERFACE    (0x00,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x04,          /* bNumEndpoints.                   */
                         0xFF,          /* bInterfaceClass                  */
                         0x00,          /* bInterfaceSubClass               */
                         0x00,          /* bInterfaceProtocol               */
                         0x00),         /* iInterface.                      */
  /* Endpoint 1 Descriptor. 7 bytes */
  USB_DESC_ENDPOINT     (USB_ENDPOINT_OUT(USBD1_DATA_OUT_EP), /* bEndpointAddress. OUT */
                         0x01,          /* bmAttributes (Isochronous).      */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x01),         /* bInterval.                       */
  /* Endpoint 2 Descriptor. 7 bytes */
  USB_DESC_ENDPOINT     (USB_ENDPOINT_IN(USBD1_DATA_IN_EP),  /* bEndpointAddress. IN */
                         0x01,          /* bmAttributes (Isochronous).      */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x01),         /* bInterval.                       */
#if BMC_USE_USB_EXTRA_ENDPOINTS
  /* Endpoint 3 Descriptor. 7 bytes */
  USB_DESC_ENDPOINT     (USB_ENDPOINT_OUT(USBD1_INTR_OUT_EP), /* bEndpointAddress. OUT */
                         0x03,          /* bmAttributes (Interrupt).      */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x02),         /* bInterval.                       */
  /* Endpoint 4 Descriptor. 7 bytes */
  USB_DESC_ENDPOINT     (USB_ENDPOINT_IN(USBD1_INTR_IN_EP),  /* bEndpointAddress. IN */
                         0x03,          /* bmAttributes (Interrupt).      */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x02),          /* bInterval.                       */
#endif

};

/*
 * Configuration Descriptor wrapper.
 */
static const USBDescriptor vcom_configuration_descriptor = {
  sizeof vcom_configuration_descriptor_data,
  vcom_configuration_descriptor_data
};

/*
 * U.S. English language identifier.
 */
static const uint8_t vcom_string0[] = {
  USB_DESC_BYTE(4),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  USB_DESC_WORD(0x0409)                 /* wLANGID (U.S. English).          */
};

/*
 * Vendor string.
 */
static const uint8_t vcom_string1[28] = {
  USB_DESC_BYTE(28),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'R', 0,
  'e', 0,
  'a', 0,
  'c', 0,
  't', 0,
  ' ', 0,
  'A', 0,
  'I', 0,
  ' ', 0,
  'L', 0,
  't', 0,
  'd', 0,
  '.', 0,
};

/*
 * Device Description string.
 */
static const uint8_t vcom_string2[60] = {
  USB_DESC_BYTE(60),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'B', 0,
  'r', 0,
  'u', 0,
  's', 0,
  'h', 0,
  'l', 0,
  'e', 0,
  's', 0,
  's', 0,
  ' ', 0,
  'M', 0,
  'o', 0,
  't', 0,
  'o', 0,
  'r', 0,
  ' ', 0,
  'C', 0,
  'o', 0,
  'n', 0,
  't', 0,
  'r', 0,
  'o', 0,
  'l', 0,
  'l', 0,
  'e', 0,
  'r', 0,
  ' ', 0,
  'V', 0,
  '2', 0
};

/*
 * Serial Number string.
 */
static const uint8_t vcom_string3[36] = {
  USB_DESC_BYTE(36),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'r', 0,
  'e', 0,
  'a', 0,
  'c', 0,
  't', 0,
  'a', 0,
  'i', 0,
  '.', 0,
  'c', 0,
  'o', 0,
  'm', 0,
  ':', 0,
  'B', 0,
  'M', 0,
  'C', 0,
  'V', 0,
  '2', 0
};

/*
 * Strings wrappers array.
 */
static const USBDescriptor vcom_strings[] = {
  {sizeof vcom_string0, vcom_string0},
  {sizeof vcom_string1, vcom_string1},
  {sizeof vcom_string2, vcom_string2},
  {sizeof vcom_string3, vcom_string3}
};

/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */
static const USBDescriptor *get_descriptor(USBDriver *usbp,
                                           uint8_t dtype,
                                           uint8_t dindex,
                                           uint16_t lang) {

  (void)usbp;
  (void)lang;
  switch (dtype) {
  case USB_DESCRIPTOR_DEVICE:
    return &vcom_device_descriptor;
  case USB_DESCRIPTOR_CONFIGURATION:
    return &vcom_configuration_descriptor;
  case USB_DESCRIPTOR_STRING:
    if (dindex < 4)
      return &vcom_strings[dindex];
  }
  return NULL;
}




/**
 * @brief   EP1 initialisation structure IN .
 */
static const USBEndpointConfig ep1config = {
  USB_EP_MODE_TYPE_ISOC,
  NULL,
  bmcDataTransmitCallback,  // in CB
  0,                        // out CB
  0x0020,                   // Max in BW
  0,                        // Max out BW
  &ep1instate,              // In state
  0,                        // Out state
  1,
  NULL
};

/**
 * @brief   EP2 initialisation structure OUT.
 */
static const USBEndpointConfig ep2config = {
  USB_EP_MODE_TYPE_ISOC,
  NULL,
  0,                         // In CB
  bmcDataReceivedCallback,   // Out CB
  0,                         // Max In
  0x0020,                    // Max Out
  0,                         // In state
  &ep2outstate,              // Out state
  1,
  NULL
};

#if BMC_USE_USB_EXTRA_ENDPOINTS
/**
 * @brief   EP3 initialisation structure IN .
 */
static const USBEndpointConfig ep3config = {
  USB_EP_MODE_TYPE_INTR,
  NULL,
  bmcIntrTransmitCallback,  // in CB
  0,                        // out CB
  0x0020,                   // Max in BW
  0,                        // Max out BW
  &ep3instate,              // In state
  0,                        // Out state
  1,
  NULL
};

/**
 * @brief   EP4 initialisation structure OUT.
 */
static const USBEndpointConfig ep4config = {
  USB_EP_MODE_TYPE_INTR,
  NULL,
  0,                         // In CB
  bmcIntrReceivedCallback,   // Out CB
  0,                         // Max In
  0x0020,                    // Max Out
  0,                         // In state
  &ep4outstate,              // Out state
  1,
  NULL
};
#endif

/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event) {

  switch (event) {
  case USB_EVENT_ADDRESS:
    return;
  case USB_EVENT_CONFIGURED:
    chSysLockFromISR();

    /* Enables the endpoints specified in the configuration.
       Note, this callback is invoked from an ISR so I-Class functions
       must be used.*/
    usbInitEndpointI(usbp, USBD1_DATA_IN_EP, &ep1config);
    usbInitEndpointI(usbp, USBD1_DATA_OUT_EP, &ep2config);
#if BMC_USE_USB_EXTRA_ENDPOINTS
    usbInitEndpointI(usbp, USBD1_INTR_IN_EP, &ep3config);
    usbInitEndpointI(usbp, USBD1_INTR_OUT_EP, &ep4config);
#endif
    /* Resetting the state of the subsystem.*/
    bmcConfigureHookI(usbp);

    chSysUnlockFromISR();
    return;
  case USB_EVENT_RESET:
    /* Falls into.*/
  case USB_EVENT_UNCONFIGURED:
    /* Falls into.*/
  case USB_EVENT_SUSPEND:
    chSysLockFromISR();

    /* Disconnection event on suspend.*/
    /* Make sure no transfers are queued. */
    bmcSuspendHookI(usbp);

    chSysUnlockFromISR();
    return;
  case USB_EVENT_WAKEUP:
    chSysLockFromISR();

    /* Connection event .*/
    /* Allow transfers to be queued. */
    bmcWakeupHookI(usbp);

    chSysUnlockFromISR();
    return;
  case USB_EVENT_STALLED:
    return;
  }
  return;
}


/*
 * Handles the USB driver global events.
 */
static void sof_handler(USBDriver *usbp) {

  (void)usbp;

  osalSysLockFromISR();

  bmcSOFHookI(usbp);

  osalSysUnlockFromISR();
}

/*
 * USB driver configuration.
 */
const USBConfig usbcfg = {
  usb_event,
  get_descriptor,
  bmcRequestsHook,
  sof_handler
};




#endif
