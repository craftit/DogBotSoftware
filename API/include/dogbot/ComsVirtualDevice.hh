#ifndef DOGBOG_ComsVirtualDevice_HEADER
#define DOGBOG_ComsVirtualDevice_HEADER 1

#include "dogbot/Coms.hh"

namespace DogBotN {


  //! Virtual device
  //! As this is based on the 'ComsC' class the roles of 'SendPacket' and 'ProcessPacket' get a bit confused.
  //!
  //! Methods that would on a normal client device be called 'SendPacket' become 'VirtSendPacket'.
  //! The 'SendPacketWire' method that would normally send data over a communications link calls

  class ComsVirtualDeviceC
   : public ComsC
  {
  public:
    //! Default
    ComsVirtualDeviceC();

    //! Destructor
    // Disconnects and closes file descriptors
    virtual ~ComsVirtualDeviceC();

    //! Open a port.
    virtual bool Open(const std::string &portAddr) override;

    //! Close connection
    virtual void Close() override;

    //! Is connection ready ?
    virtual bool IsReady() const override;

    //! Send packet
    virtual void SendPacketWire(const uint8_t *data,int len) override;

    //! Set the device type to use
    void SetDeviceType(enum DeviceTypeT deviceType);

    //! Access the type of this device
    enum DeviceTypeT DeviceType() const
    { return m_deviceType; }

  protected:


    //! Handle incoming packet to device
    virtual void VirtProcessPacket(const uint8_t *data,int len);

    //! Send packet out from device
    virtual bool VirtSendPacket(const uint8_t *data,int len);

    //! Send error
    void VirtSendError(
        uint8_t deviceId,
        ComsErrorTypeT code,
        uint8_t originalPacketType,
        uint8_t data
        );

    //! Set a parameter
    virtual bool SetParam(enum ComsParameterIndexT index,union BufferTypeT *data,int len);

    //! Read a parameter
    virtual bool ReadParam(enum ComsParameterIndexT index,int *len,union BufferTypeT *data);

    //! Read a parameter and send it
    bool ReadParamAndReply(enum ComsParameterIndexT paramIndex);

    //! Send parameter and report and error if it fails
    void VirtSendParamUpdate(enum ComsParameterIndexT paramIndex);

    //! Send a sync message.
    void VirtSendSync();

    //! Send a message string.
    void VirtSendMessage(const char *msg);

    //! Change control state of device.
    virtual bool ChangeControlState(enum ControlStateT newState,enum StateChangeSourceT changeSource);


    bool m_indicator = false;

    enum DeviceTypeT m_deviceType;
    uint32_t m_nodeUId1 = 0x01234567;
    uint32_t m_nodeUId2 = 0x89012346;
    uint8_t m_deviceId = 0;
    enum ControlStateT m_controlState = CS_StartUp;
    enum FaultCodeT m_lastFaultCode =  FC_Ok;

    bool m_indicatorState = false;
    uint32_t m_faultState = 0;


  };
}
#endif

