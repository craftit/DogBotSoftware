
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

#define DATA_IN_EP           1
#define DATA_OUT_EP          2

namespace DogBotN
{

  static void usbTransferCB(struct libusb_transfer *transfer)
  {
    USBTransferDataC *transferData = (USBTransferDataC *)transfer->user_data;

    if(transfer->endpoint == DATA_IN_EP | 0x80 )
      transferData->ComsUSB()->ProcessInTransfer(transferData);

    if(transfer->endpoint == DATA_OUT_EP)
      transferData->ComsUSB()->ProcessOutTransfer(transferData);
  }

  USBTransferDataC::USBTransferDataC()
  {}

  USBTransferDataC::~USBTransferDataC()
  {
    if(m_transfer != 0)
      libusb_free_transfer(m_transfer);
  }

  void USBTransferDataC::Setup(ComsUSBC *coms,struct libusb_device_handle *handle,USBTransferDirectionT direction)
  {
    m_direction = direction;
    m_comsUSB = coms;

    memset(m_buffer,0,sizeof(m_buffer));
    // Initialise an input transfer.
    m_transfer = libusb_alloc_transfer(1);

    m_transfer->dev_handle = handle;
    if(direction == UTD_IN)
      m_transfer->endpoint = DATA_IN_EP | 0x80;
    else
      m_transfer->endpoint = DATA_OUT_EP;
    m_transfer->flags = 0;
    m_transfer->type = LIBUSB_TRANSFER_TYPE_ISOCHRONOUS;
    m_transfer->timeout = 0;
    m_transfer->length = 64;
    m_transfer->actual_length = 0;
    m_transfer->callback = usbTransferCB;
    m_transfer->user_data = this;
    m_transfer->buffer = m_buffer;
    m_transfer->actual_length = sizeof(m_buffer);
    m_transfer->num_iso_packets = 1;
    m_transfer->iso_packet_desc[0].actual_length = 64;
    m_transfer->iso_packet_desc[0].length = 64;
    m_transfer->iso_packet_desc[0].status = LIBUSB_TRANSFER_ERROR;
  }



  // ====================================================================

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
    m_terminate = true;

    if(libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
      libusb_hotplug_deregister_callback(m_usbContext,m_hotplugCallbackHandle);
    }
    if(m_handle != 0) {
      if(m_claimedInferface)
        libusb_release_interface(m_handle, 0);

      libusb_close(m_handle);
      m_handle = 0;
    }

    if(m_threadUSB.joinable())
      m_threadUSB.join();

    libusb_exit(m_usbContext);
    m_usbContext = 0;
  }


  static int coms_usb_hotplug_callback(libusb_context *ctx, libusb_device *device, libusb_hotplug_event event, void *user_data)
  {
    if(event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED)
      ((ComsUSBC *) user_data)->HotPlugArrivedCallback(device,event);
    if(event == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT)
      ((ComsUSBC *) user_data)->HotPlugDepartedCallback(device,event);
    return 0;
  }

  void ComsUSBC::HotPlugArrivedCallback(libusb_device *device, libusb_hotplug_event event)
  {
    m_log->info("Got hotplug event {} ",(int) event);
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
    std::string serialNumber((char *)buff);
    m_log->info("SerialNumber:{}  ",serialNumber);
    if(serialNumber != "reactai.com:BMCV2") {
      // Not our device.  Close handle.
      libusb_close(m_handle);
      m_handle = 0;
      return ;
    }

    Open(m_handle);
  }

  //! Open connection to device
  void ComsUSBC::Open(struct libusb_device_handle *handle)
  {
    m_log->info("BMCV2 device open ");

    int rc = libusb_claim_interface(handle, 0);
    if (rc != LIBUSB_SUCCESS) {
      m_log->error("Failed to claim interface. {} ",libusb_error_name(rc));
      return ;
    }
    m_claimedInferface = true;

    m_inTransfers = std::vector<USBTransferDataC>(2);
    m_outTransfers = std::vector<USBTransferDataC>(16);

    // Setup a series of IN transfers.
    for(auto &a : m_inTransfers) {
      a.Setup(this,handle,UTD_IN);
#if 1
      int rc = libusb_submit_transfer(a.Transfer());
      if(rc != LIBUSB_SUCCESS) {
        m_log->error("Got error setting up input transfer. {} ",libusb_error_name(rc));
      } else {
        m_log->debug("Input transfer setup ok. ");
      }
#endif
    }

    // Setup some OUT transfers.
    for(auto &a : m_outTransfers) {
      a.Setup(this,handle,UTD_OUT);
      m_outFree.push_back(&a);
    }

  }


  //! Process incoming data.

  void ComsUSBC::ProcessInTransfer(USBTransferDataC *data)
  {
    //m_log->info("ProcessTransferIn");
    switch(data->Transfer()->status)
    {
      case LIBUSB_TRANSFER_COMPLETED: {
        //m_log->info("Transfer completed");
        int at = 0;
        for(int i = 0;i < data->Transfer()->num_iso_packets;i++) {
          int len = data->Transfer()->iso_packet_desc[i].actual_length;
          if(len == 0)
            continue;
          unsigned char *pdata = &data->Transfer()->buffer[at];
          m_log->info("Packet size {} Type:{} Status:{}",len,(int) pdata[0],(int) data->Transfer()->iso_packet_desc[i].status);
          if(len > 0) {
            ProcessPacket(pdata,len);
            at += len;
          }
        }
      } break;
      case LIBUSB_TRANSFER_NO_DEVICE:
        m_log->warn("Device removed.");
        break;
      case LIBUSB_TRANSFER_OVERFLOW:
      default:
        m_log->warn("Transfer status {} ",data->Transfer()->status);
        break;
    }

    // Clear things out for debug.
    memset(data->Transfer()->buffer,0,data->Transfer()->length);
    for(int i = 0;i < data->Transfer()->num_iso_packets;i++) {
      data->Transfer()->iso_packet_desc[i].actual_length = 0;
      data->Transfer()->iso_packet_desc[i].length = data->BufferSize();
      data->Transfer()->iso_packet_desc[i].status = LIBUSB_TRANSFER_ERROR;
    }

    // Resubmit transfer to get more lovely data.
    libusb_submit_transfer(data->Transfer());
  }

  //! Process outgoing data complete

  void ComsUSBC::ProcessOutTransfer(USBTransferDataC *data)
  {
    m_log->info("ProcessTransferOut");

    std::lock_guard<std::mutex> lock(m_accessTx);
    m_outFree.push_back(data);
  }

  //! Handle hot plug callback.
  void ComsUSBC::HotPlugDepartedCallback(libusb_device *device, libusb_hotplug_event event)
  {
    m_log->info("Device left ");
  }


  void ComsUSBC::Init()
  {
    //putenv("LIBUSB_DEBUG=4");
    int r = libusb_init(&m_usbContext);
    if(r != 0) {
      m_log->error("Failed to create libusb context {} : {} ",r,libusb_error_name(r));
      return ;
    }

    m_inTransfers = std::vector<USBTransferDataC>(16);
    m_outTransfers = std::vector<USBTransferDataC>(16);

    //libusb_set_debug(m_usbContext,LIBUSB_LOG_LEVEL_DEBUG);
    if(libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
      m_log->info("usb hotplug notifications are available");

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
        m_log->error("Failed to register hotplug callback.{}",libusb_error_name(ret));
      }

    } else {
      m_log->info("usb hotplug notifications are not available");
    }

    m_threadUSB = std::move(std::thread { [this]{ RunUSB(); } });



  }

  //! Close connection
  void ComsUSBC::Close()
  {
    m_log->debug("Close called.");
    m_terminate = true;
    if(!m_mutexExitOk.try_lock_for(std::chrono::milliseconds(500))) {
      m_log->error("Failed to shutdown receiver thread.");
    }
    m_threadUSB.join();
  }

  //! Is connection ready ?
  bool ComsUSBC::IsReady() const
  {
    return false; //m_fd >= 0;
  }


  bool ComsUSBC::Open(const std::string &portAddr)
  {
    // If we're connecting over the serial port we want to be in bridge mode.
    //SendEnableBridge(true);
    return true;
  }

  //! Process received packet.

  bool ComsUSBC::RunUSB()
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
    if(m_outFree.empty()) {
      m_log->error("Dropping packet, no free transmit buffers.");
      return ;
    }
    if(len >= 64) {
      m_log->error("Dropping large packet.");
      return ;
    }
    USBTransferDataC *txBuffer;

    {
      std::lock_guard<std::mutex> lock(m_accessTx);
      txBuffer = m_outFree.back();
      m_outFree.pop_back();
    }

    memcpy(txBuffer->Buffer(),buff,len);
    txBuffer->Transfer()->iso_packet_desc[0].actual_length = len;
    txBuffer->Transfer()->iso_packet_desc[0].length = len;
    int rc = libusb_submit_transfer(txBuffer->Transfer());
    if(rc != LIBUSB_SUCCESS) {
      m_log->error("Got error setting up input transfer. {} ",libusb_error_name(rc));
    } else {
      m_log->debug("Input transfer setup ok. ");
    }

  }

}
