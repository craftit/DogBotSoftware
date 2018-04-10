#ifndef DOGBOG_COMSUSB_HEADER
#define DOGBOG_COMSUSB_HEADER 1

#include "dogbot/Coms.hh"
#include <libusb.h>
#include <deque>

namespace DogBotN {

  class ComsUSBC;

  //! Direction for USB transfer, used in USBTransferDataC.

  enum USBTransferDirectionT {
    UTD_IN,
    UTD_OUT
  };

  //! Simple packet structure.
  class DataPacketT {
  public:
    int m_len;
    uint8_t m_data[64];
  };


  //! Information about a transfer

  class USBTransferDataC
  {
  public:
    //! Construct a blank transfer structure.
    //! You must call SetupIso or SetupIntr before using.
    USBTransferDataC();

    //! Free m_transfer structure.
    ~USBTransferDataC();

    //! Setup an ISO buffer
    void SetupIso(ComsUSBC *coms,struct libusb_device_handle *handle,USBTransferDirectionT direction);

    //! Setup an Ctrl buffer
    void SetupIntr(ComsUSBC *coms,struct libusb_device_handle *handle,USBTransferDirectionT direction);

    //! Get size of buffer.
    unsigned BufferSize() const
    { return sizeof(m_buffer); }

    //! Access buffer.
    unsigned char *Buffer()
    { return m_buffer; }

    USBTransferDirectionT Direction() const
    { return m_direction; }

    ComsUSBC *ComsUSB()
    { return m_comsUSB; }

    struct libusb_transfer* Transfer()
    { return m_transfer; }

    //! Test if the object if for a particular device.
    bool IsForUSBDevice(struct libusb_device_handle *handle) const;
  protected:
    ComsUSBC *m_comsUSB = 0;
    USBTransferDirectionT m_direction = UTD_IN;
    struct libusb_transfer* m_transfer = 0;
    unsigned char m_buffer[64];
  };

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
    virtual void SendPacketWire(const uint8_t *data,int len) override;

    //! Handle hot plug callback.
    void HotPlugArrivedCallback(libusb_device *device, libusb_hotplug_event event);

    //! Handle hot plug callback.
    void HotPlugDepartedCallback(libusb_device *device, libusb_hotplug_event event);

    //! Process incoming data.
    void ProcessInTransferIso(USBTransferDataC *data);

    //! Process outgoing data complete
    void ProcessOutTransferIso(USBTransferDataC *data);

  protected:
    //! Close usb handle
    void CloseUSB();

    // 'data' is a free buffer or null.
    void SendTxQueue(USBTransferDataC *data);

    void Init();

    //! Open connection to device
    void Open(struct libusb_device_handle *handle);

    //! Remove transfer from active list.
    void TransferComplete(USBTransferDataC *data,int typeId);

    std::vector<USBTransferDataC *> m_outDataFree;
    std::vector<USBTransferDataC *> m_activeTransfers;

    struct libusb_device_handle *m_handle = 0;
    struct libusb_device *m_device = 0;

    bool m_claimedInferface = false;
    int m_state = 0;

    struct libusb_context *m_usbContext = 0;

    libusb_hotplug_callback_handle m_hotplugCallbackHandle = 0;

    bool RunUSB();

    std::thread m_threadUSB;

    std::deque<DataPacketT> m_txQueue;

    std::mutex m_accessTx;
    std::timed_mutex m_mutexExitOk;

  };
}
#endif
