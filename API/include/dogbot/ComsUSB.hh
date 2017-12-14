#ifndef DOGBOG_COMSUSB_HEADER
#define DOGBOG_COMSUSB_HEADER 1

#include "dogbot/Coms.hh"
#include <libusb.h>

namespace DogBotN {

  //! Low level serial communication over usb with the driver board

  class ComsUSBC
   : public ComsC
  {
  public:
    ComsUSBC(std::shared_ptr<spdlog::logger> &log);

    //! default
    ComsUSBC();

    //! Destructor
    // Disconnects and closes file descriptors
    virtual ~ComsUSBC();

    //! Open a port.
    virtual bool Open(const std::string &portAddr) override;

    //! Close connection
    virtual void Close() override;

    //! Is connection ready ?
    virtual bool IsReady() const override;

    //! Send packet
    virtual void SendPacket(const uint8_t *data,int len) override;

    //! Handle hot plug callback.
    void HotPlugCallback(libusb_device *device, libusb_hotplug_event event);

  protected:
    void Init();

    struct libusb_device_handle *m_handle = 0;

    int m_state = 0;

    struct libusb_context *m_usbContext = 0;

    libusb_hotplug_callback_handle m_hotplugCallbackHandle = 0;

    bool RunRecieve();

    std::thread m_threadRecieve;

    std::mutex m_accessTx;
    std::timed_mutex m_mutexExitOk;

  };
}
#endif
