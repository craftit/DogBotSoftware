#ifndef PACKETUSB_HEADER
#define PACKETUSB_HEADER 1

#include "hal.h"

#include "dogbot/protocol.h"


/*
 * Endpoints to be used for USBD1.
 */
#define USBD1_DATA_IN_EP     BMCUSB_DATA_IN_EP
#define USBD1_DATA_OUT_EP    BMCUSB_DATA_OUT_EP

#if BMC_USE_USB_EXTRA_ENDPOINTS
#define USBD1_INTR_IN_EP     BMCUSB_INTR_IN_EP
#define USBD1_INTR_OUT_EP    BMCUSB_INTR_OUT_EP
#endif

/*
 * Endpoints to be used for USBD1.
 *  IN to the host (tx)
 *  OUT from the host (rx)
 */

#ifdef __cplusplus
extern "C" {
#endif
  extern const USBConfig usbcfg;

  void bmcDataTransmitCallback(USBDriver *usbp, usbep_t ep);

  void bmcDataReceivedCallback(USBDriver *usbp, usbep_t ep);

  void bmcIntrTransmitCallback(USBDriver *usbp, usbep_t ep);

  void bmcIntrReceivedCallback(USBDriver *usbp, usbep_t ep);

  bool bmcRequestsHook(USBDriver *usbp);

  bool bmcSOFHookI(USBDriver *usbp);

  void bmcWakeupHookI(USBDriver *usbp);

  void bmcSuspendHookI(USBDriver *usbp);

  void bmcConfigureHookI(USBDriver *usbp);

#ifdef __cplusplus
}
#endif

#endif
