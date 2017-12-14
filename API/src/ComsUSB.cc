
#include <sstream>

#include <thread>
#include <vector>
#include <functional>
#include <assert.h>
#include <mutex>

#include <stdio.h>
#include <string.h>

#include <fstream>
#include <iostream>
#include <fcntl.h>

#include <unistd.h>

#include "dogbot/ComsUSB.hh"

namespace DogBotN
{

  ComsUSBC::ComsUSBC(std::shared_ptr<spdlog::logger> &log)
   : ComsC(log)
  {
    Init();
  }

  //! default
  ComsUSBC::ComsUSBC()
  {
    Init();
  }

  // Disconnects and closes file descriptors
  ComsUSBC::~ComsUSBC()
  {
    if(libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
      libusb_hotplug_deregister_callback(m_usbContext,m_hotplugCallbackHandle);
    }

    libusb_exit(m_usbContext);
    m_usbContext = 0;
  }


  static int coms_usb_hotplug_callback(libusb_context *ctx, libusb_device *device, libusb_hotplug_event event, void *user_data)
  {
    ((ComsUSBC *) user_data)->HotPlugCallback(device,event);
    return 0;
  }

  void ComsUSBC::HotPlugCallback(libusb_device *device, libusb_hotplug_event event)
  {
    m_log->info("Got hotplug event {} ",(int) event);
    if(event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) {
      m_log->info("Device arrived ");

      struct libusb_device_descriptor desc;
      int rc;

      rc = libusb_get_device_descriptor(device, &desc);
      if (LIBUSB_SUCCESS != rc) {
        m_log->error("Error getting device descriptor");
        return ;
      }

      if(m_handle != 0) {
        m_log->info("Drive already open. Ignoring device. ");
        return ;
      }
      if((rc = libusb_open(device,&m_handle)) != LIBUSB_SUCCESS) {
        m_log->error("Error opening device. {} ",libusb_error_name(rc));
        return ;
      }

      m_log->info("Device attached: {}:{} ", desc.idVendor, desc.idProduct);

      unsigned char buff[128];
      if((rc = libusb_get_string_descriptor_ascii(m_handle, desc.iSerialNumber, buff, 128)) < 0) {
        m_log->error("Error getting serial number. {} ",libusb_error_name(rc));
        libusb_close(m_handle);
        m_handle = 0;
        return ;
      }

      m_log->info("SerialNumber:{}  ",buff);
      libusb_close(m_handle);
      m_handle = 0;
    }
    if(event == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT) {
      m_log->info("Device left ");
    }



  }


  void ComsUSBC::Init()
  {
    //putenv("LIBUSB_DEBUG=4");
    int r = libusb_init(&m_usbContext);
    if(r != 0) {
      m_log->error("Failed to create libusb context {} : {} ",r,libusb_error_name(r));
      return ;
    }
    libusb_set_debug(m_usbContext,LIBUSB_LOG_LEVEL_DEBUG);
    if(libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
      m_log->info("usb hotplug notifcations are available");

      int ret = libusb_hotplug_register_callback(
          m_usbContext,
          (libusb_hotplug_event) (LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT),
          LIBUSB_HOTPLUG_ENUMERATE,
          0x27d8,
          0x16c0,
          LIBUSB_HOTPLUG_MATCH_ANY,
          &coms_usb_hotplug_callback,
          this,
          &m_hotplugCallbackHandle
      );
      if(ret != LIBUSB_SUCCESS) {
        m_log->error("Failed to registed hotplug callback.{}",libusb_error_name(ret));
      }

    } else {
      m_log->info("usb hotplug notifcations are not available");
    }

  }

  //! Close connection
  void ComsUSBC::Close()
  {
    m_log->debug("Close called.");
    m_terminate = true;
    if(!m_mutexExitOk.try_lock_for(std::chrono::milliseconds(500))) {
      m_log->error("Failed to shutdown receiver thread.");
    }
    m_threadRecieve.join();
  }

  //! Is connection ready ?
  bool ComsUSBC::IsReady() const
  {
    return false; //m_fd >= 0;
  }


  bool ComsUSBC::Open(const std::string &portAddr)
  {
    m_terminate = false;
    m_log->info("Opening: '{}'",portAddr);
    //.....
    if(!m_terminate) {
      if(!m_mutexExitOk.try_lock()) {
        m_log->error("Exit lock already locked, multiple threads attempting to open coms ?");
      } else {
        m_threadRecieve = std::move(std::thread { [this]{ RunRecieve(); } });
      }
    }
    // If we're connecting over the serial port we want to be in bridge mode.
    SendEnableBridge(true);
    return true;
  }

  //! Process received packet.

  bool ComsUSBC::RunRecieve()
  {
    m_log->debug("Running receiver.");
    int rc = 0;
    while(!m_terminate) {
      if((rc = libusb_handle_events(m_usbContext)) < 0) {
        m_log->error("libusb_handle_events() failed: {}",libusb_error_name(rc));
      }

    }
    m_mutexExitOk.unlock();
    m_log->debug("Exiting receiver. ");

    return true;
  }

  //! Send packet
  void ComsUSBC::SendPacket(const uint8_t *buff,int len)
  {
  }

}
