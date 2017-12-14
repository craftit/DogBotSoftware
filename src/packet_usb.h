#ifndef PACKETUSB_HEADER
#define PACKETUSB_HEADER 1

#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif
  void bmcDataTransmitCallback(USBDriver *usbp, usbep_t ep);

  void bmcDataReceivedCallback(USBDriver *usbp, usbep_t ep);

  bool bmcRequestsHook(USBDriver *usbp);

  bool bmcSOFHookI(USBDriver *usbp);



#ifdef __cplusplus
}
#endif

#endif
