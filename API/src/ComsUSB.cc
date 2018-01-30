
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

#define DODEBUG 0
#if DODEBUG
#define ONDEBUG(x) x
#else
#define ONDEBUG(x)
#endif

namespace DogBotN
{

  static void usbTransferCB(struct libusb_transfer *transfer)
  {
    USBTransferDataC *transferData = (USBTransferDataC *)transfer->user_data;

    int endPoint = transfer->endpoint & 0x0F;

    if(endPoint == BMCUSB_DATA_IN_EP )
      transferData->ComsUSB()->ProcessInTransferIso(transferData);
    else if(endPoint == BMCUSB_DATA_OUT_EP)
      transferData->ComsUSB()->ProcessOutTransferIso(transferData);
#if BMC_USE_USB_EXTRA_ENDPOINTS
    else
      if(endPoint == BMCUSB_INTR_IN_EP )
      transferData->ComsUSB()->ProcessInTransferIntr(transferData);
    else if(endPoint == BMCUSB_INTR_OUT_EP)
      transferData->ComsUSB()->ProcessOutTransferIntr(transferData);
#endif
    else
    {
      std::cerr << "Unexpected end point " << endPoint << std::endl;
    }
  }

  USBTransferDataC::USBTransferDataC()
  {}

  USBTransferDataC::~USBTransferDataC()
  {
    if(m_transfer != 0)
      libusb_free_transfer(m_transfer);
    m_transfer = 0;
  }

  void USBTransferDataC::SetupIso(
      ComsUSBC *coms,
      struct libusb_device_handle *handle,
      USBTransferDirectionT direction)
  {
    m_direction = direction;
    m_comsUSB = coms;

    memset(m_buffer,0,sizeof(m_buffer));
    // Initialise an input transfer.
    m_transfer = libusb_alloc_transfer(1);

    unsigned char endPoint;

    if(direction == UTD_IN)
      endPoint = BMCUSB_DATA_IN_EP | 0x80;
    else
      endPoint = BMCUSB_DATA_OUT_EP;
#if 1
    libusb_fill_iso_transfer(
        m_transfer,
        handle,
        endPoint,
        m_buffer,
        sizeof(m_buffer),
        1,
        &usbTransferCB,
        this,
        0
        );
#else
    m_transfer->dev_handle = handle;
    if(direction == UTD_IN)
      m_transfer->endpoint = BMCUSB_DATA_IN_EP | 0x80;
    else
      m_transfer->endpoint = BMCUSB_DATA_OUT_EP;
    m_transfer->flags = 0;
    m_transfer->type = LIBUSB_TRANSFER_TYPE_ISOCHRONOUS;
    m_transfer->timeout = 0;
    m_transfer->callback = usbTransferCB;
    m_transfer->user_data = this;
    m_transfer->buffer = m_buffer;
    m_transfer->length = sizeof(m_buffer);
    m_transfer->actual_length = sizeof(m_buffer);
#endif
    m_transfer->num_iso_packets = 1;
    m_transfer->iso_packet_desc[0].actual_length = sizeof(m_buffer);
    m_transfer->iso_packet_desc[0].length = sizeof(m_buffer);
    m_transfer->iso_packet_desc[0].status = LIBUSB_TRANSFER_COMPLETED;
  }

  //! Test if the object if for a particular device.
  bool USBTransferDataC::IsForUSBDevice(struct libusb_device_handle *handle) const
  {
    if(m_transfer == 0)
      return false;
    return m_transfer->dev_handle == handle;
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

  //! Close usb handle
  void ComsUSBC::CloseUSB()
  {
    if(m_handle == 0)
      return ;

    std::lock_guard<std::mutex> lock(m_accessTx);

    // Dump all packets from tx queue.
    m_txQueue.empty();

    // Cancel all active transfers.
    std::vector<USBTransferDataC *> stillActive;
    stillActive.reserve(32);
    for(auto a : m_activeTransfers) {
      assert(a->Transfer() != 0);
      int rc = libusb_cancel_transfer(a->Transfer());
      if(rc == 0) {
        stillActive.push_back(a);
      } else {
        if(rc != LIBUSB_ERROR_NOT_FOUND) {
          m_log->error("Unexpected error cancelling transfer {} ",rc);
        }
        delete a;
        continue;
      }
    }
    m_activeTransfers = stillActive;

    // Free all waiting buffers
    while(m_outDataFree.size() > 0) {
      delete m_outDataFree.back();
      m_outDataFree.pop_back();
    }

    if(m_claimedInferface) {
      libusb_release_interface(m_handle, 0);
      m_claimedInferface = false;
    }


    libusb_close(m_handle);
    m_handle = 0;
    m_device = 0;
  }

  // Disconnects and closes file descriptors
  ComsUSBC::~ComsUSBC()
  {
    m_terminate = true;

    if(libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
      libusb_hotplug_deregister_callback(m_usbContext,m_hotplugCallbackHandle);
    }

    CloseUSB();

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

  //! Handle hot plug callback.
  void ComsUSBC::HotPlugDepartedCallback(libusb_device *device, libusb_hotplug_event event)
  {
    if(m_device != device) {
      m_log->info("Another device departed. ");
      return ;
    }
    m_log->info("Device departed. ");

    CloseUSB();
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

    m_device = device;

    Open(m_handle);
  }

  //! Open connection to device
  void ComsUSBC::Open(struct libusb_device_handle *handle)
  {
    m_log->info("BMCV2 device open ");

    int rc = libusb_claim_interface(handle, 0);
    if (rc != LIBUSB_SUCCESS) {
      m_log->error("Failed to claim interface. {},  Is there another client using the connection? ",libusb_error_name(rc));
      libusb_close(m_handle);
      m_handle = 0;
      return ;
    }
    m_claimedInferface = true;

    {
      std::lock_guard<std::mutex> lock(m_accessTx);

      m_log->info("Active transfers on open: {} ",m_activeTransfers.size());

      // Setup a series of IN transfers.
      for(int i = 0;i < 2;i++) {
        USBTransferDataC *a = new USBTransferDataC();
        a->SetupIso(this,handle,UTD_IN);
        int rc = libusb_submit_transfer(a->Transfer());
        if(rc != LIBUSB_SUCCESS) {
          m_log->error("Got error setting up input iso transfer. {} ",libusb_error_name(rc));
          delete a;
          assert(0); // We need to deal with this.
        } else {
          m_activeTransfers.push_back(a);
          m_log->debug("Input iso transfer setup ok. ");
        }
      }

      m_outDataFree.empty();
      m_outDataFree.reserve(4);
      // Setup some OUT transfers.
      for(int i = 0;i < 4;i++) {
        USBTransferDataC *a = new USBTransferDataC();
        a->SetupIso(this,handle,UTD_OUT);
        m_outDataFree.push_back(a);
      }

    }


    // From USB we're always in bridge mode.
    for(int i = 0;i < 3;i++)
      SendEnableBridge(true);
  }

  //! Remove transfer from active list.
  void ComsUSBC::TransferComplete(USBTransferDataC *data)
  {
    auto at = std::find(m_activeTransfers.begin(),m_activeTransfers.end(),data);
    if(at == m_activeTransfers.end()) {
      m_log->error("Transfer not found, can't be completed.");
      return ;
    }
    *at = *(m_activeTransfers.end()-1);
    m_activeTransfers.pop_back();
  }

  //! Process incoming data.

  void ComsUSBC::ProcessInTransferIso(USBTransferDataC *data)
  {
    if(!data->IsForUSBDevice(m_handle)) {
      m_log->info("Dropping in transfer from old device. ");
      TransferComplete(data);
      delete data;
      return ;
    }

    //m_log->info("ProcessTransferIn");
    switch(data->Transfer()->status)
    {
      case LIBUSB_TRANSFER_COMPLETED: {
        //m_log->info("Transfer completed");
        int at = 0;
        for(int i = 0;i < data->Transfer()->num_iso_packets;i++) {
          int usbLen = data->Transfer()->iso_packet_desc[i].actual_length;
          if(usbLen <= 0)
            continue;
          unsigned char *pdata = &data->Transfer()->buffer[at];
          int dat = 0;
          //ONDEBUG(m_log->info("Packet iso size {} Type:{} Status:{}",usbLen,(int) pdata[dat],(int) data->Transfer()->iso_packet_desc[i].status));
          while(dat < usbLen) {
            int packetLen = pdata[dat++];
            if(packetLen == 0) {
              m_log->error("Zero packet length at {} ",dat);
              break;
            }
            if(packetLen < 0 || packetLen > 15) {
              m_log->error("Unexpected packet size at {} of {} ",dat,packetLen);
              break;
            }
            ONDEBUG(m_log->info("Packet iso at {} Len:{} Type:{} ",dat,packetLen,(int) pdata[dat+1]));
            ProcessPacket(&pdata[dat],packetLen);
            dat += packetLen;
          }
          if(dat != usbLen) {
            m_log->error("Packet boundary error. at {} of {} ",dat,usbLen);
          }
          at += usbLen;
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

  void ComsUSBC::ProcessOutTransferIso(USBTransferDataC *data)
  {
    TransferComplete(data);
    // If not open, just drop the packet.
    if(!data->IsForUSBDevice(m_handle)) {
      m_log->info("Dropping in transfer from old device. ");
      delete data;
      return ;
    }

    SendTxQueue(data);
  }


  void ComsUSBC::Init()
  {
    //putenv("LIBUSB_DEBUG=4");
    int r = libusb_init(&m_usbContext);
    if(r != 0) {
      m_log->error("Failed to create libusb context {} : {} ",r,libusb_error_name(r));
      return ;
    }

    libusb_set_debug(m_usbContext,LIBUSB_LOG_LEVEL_INFO);
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
    return m_handle != 0;
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

    CloseUSB();

    m_mutexExitOk.unlock();
    m_log->debug("Exiting receiver. ");

    return true;
  }

  void ComsUSBC::SendTxQueue(USBTransferDataC *txBuffer)
  {
    int at = 0;

    // Gather some data to send.
    {
      std::lock_guard<std::mutex> lock(m_accessTx);
      if(m_handle == 0)
        return ;
      if(txBuffer == 0) {
        if(m_txQueue.size() == 0)
          return ; // Nothing to do.
        if(m_outDataFree.empty()) {
          //m_log->error("No free iso transmit buffers.");
          return ;
        }
        txBuffer = m_outDataFree.back();
        m_outDataFree.pop_back();
        assert(txBuffer->Transfer() != 0);
      } else {
        if(m_txQueue.size() == 0) {
          // Nothing to send, just return the buffer.
          m_outDataFree.push_back(txBuffer);
          return ;
        }
      }

      uint8_t *txData = txBuffer->Buffer();
      while(!m_txQueue.empty() && at < 50) {
        DataPacketT &packet = m_txQueue.front();
        txData[at++] = packet.m_len;
        memcpy(&txData[at],packet.m_data,packet.m_len);
        at += packet.m_len;
        m_txQueue.pop_front();
      }
    }

    // Put the packet together and transfer it.
    assert(txBuffer->Transfer() != 0);
    txBuffer->Transfer()->num_iso_packets = 1;
    txBuffer->Transfer()->iso_packet_desc[0].actual_length = at;
    txBuffer->Transfer()->iso_packet_desc[0].length = at;
    txBuffer->Transfer()->length = at;
    int rc = libusb_submit_transfer(txBuffer->Transfer());
    if(rc != LIBUSB_SUCCESS) {
      m_log->error("Got error setting up output iso transfer. {} ",libusb_error_name(rc));
      TransferComplete(txBuffer);
      m_outDataFree.push_back(txBuffer);
    } else {
      m_activeTransfers.push_back(txBuffer);
      ONDEBUG(m_log->info("Output iso transfer setup ok. "));
    }
  }

  //! Send packet
  void ComsUSBC::SendPacket(const uint8_t *buff,int len)
  {
    if(len >= 64) {
      m_log->error("Dropping large packet.");
      return ;
    }
    if(len < 1) {
      m_log->error("Dropping small packet.");
      return ;
    }
    {
      std::lock_guard<std::mutex> lock(m_accessTx);
      // If not open, just drop the packet.
      if(m_handle == 0)
        return ;
      // Is the queue too long ?
      if(m_txQueue.size() > 32) {
        m_log->error("Transmit queue to long, dropping packet. {} ",m_txQueue.size());
        return ;
      }
      if(m_txQueue.size() > 24) {
        m_log->warn("Transmit queue getting very long. {} ",m_txQueue.size());
      }
      m_txQueue.emplace_back();
      DataPacketT &packet = m_txQueue.back();
      packet.m_len = len;
      memcpy(packet.m_data,buff,len);
    }
    SendTxQueue(0);
  }

}
