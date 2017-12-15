#ifndef PACKETUSB_HEADER
#define PACKETUSB_HEADER 1

#include "hal.h"

/*
 * Endpoints to be used for USBD1.
 */
#define USBD1_DATA_IN_EP           1
#define USBD1_DATA_OUT_EP          2

/*
 * Endpoints to be used for USBD1.
 *  IN to the host (tx)
 *  OUT from the host (rx)
 */

#ifdef __cplusplus
extern "C" {
#endif
  void bmcDataTransmitCallback(USBDriver *usbp, usbep_t ep);

  void bmcDataReceivedCallback(USBDriver *usbp, usbep_t ep);

  bool bmcRequestsHook(USBDriver *usbp);

  bool bmcSOFHookI(USBDriver *usbp);

  void bmcWakeupHookI(USBDriver *usbp);

  void bmcSuspendHookI(USBDriver *usbp);

  void bmcConfigureHookI(USBDriver *usbp);

#ifdef __cplusplus
}
#endif

#endif
