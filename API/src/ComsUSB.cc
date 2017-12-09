
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
    libusb_exit(m_usbContext);
    m_usbContext = 0;
  }

  void ComsUSBC::Init()
  {
    putenv("LIBUSB_DEBUG=4");
    int r = libusb_init(&m_usbContext);
    if(r != 0) {
      m_log->error("Failed to create libusb context {} : {} ",r,libusb_error_name(r));
      return ;
    }
    libusb_set_debug(m_usbContext,LIBUSB_LOG_LEVEL_DEBUG);
    if(libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
      m_log->info("usb hotplug notifcations are available");
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
    while(!m_terminate) {
      sleep(10);
    }
    m_mutexExitOk.unlock();
    m_log->debug("Exiting receiver fd. ");

    return true;
  }

  //! Send packet
  void ComsUSBC::SendPacket(const uint8_t *buff,int len)
  {
  }

}
