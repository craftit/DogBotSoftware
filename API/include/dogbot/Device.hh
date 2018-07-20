#ifndef DOGBOG_DEVICE_HEADER
#define DOGBOG_DEVICE_HEADER 1

#include "dogbot/Coms.hh"
#include "dogbot/Common.hh"
#include "dogbot/CallbackArray.hh"
#include <jsoncpp/json/json.h>

namespace DogBotN {

  //! Base device class.

  class DeviceC
  {
  public:
    typedef std::function<void (ComsParameterIndexT parameter)> ParameterUpdateFuncT;

    DeviceC();

    // Construct from coms link and deviceId
    DeviceC(const std::shared_ptr<ComsC> &coms,int deviceId);

    // Construct with announce packet.
    DeviceC(const std::shared_ptr<ComsC> &coms,int deviceId,const PacketDeviceIdC &pktAnnounce);

    //! Destructor
    virtual ~DeviceC();

    //! Access the device type
    virtual const char *DeviceType() const;

    //! Set servo enabled flag.
    void SetEnabled(bool enabled)
    { m_enabled = enabled; }

    //! Name of device
    const std::string &DeviceName() const
    { return m_deviceName; }

    //! Set the name of the device
    void SetDeviceName(const std::string &name)
    { m_deviceName = name; }

    //! Is servo enabled ?
    bool IsEnabled() const
    { return m_enabled; }

    //! Access the device id.
    int Id() const
    { return m_id; }

    //! Set device id.
    void SetId(int id)
    { m_id = id; }

    //! Access part 1 of unique id
    int UId1() const
    { return m_uid1; }

    //! Access part 2 of unique id
    int UId2() const
    { return m_uid2; }

    //! Does this servo have a matching id ?
    bool HasUID(uint32_t uid1,uint32_t uid2) const
    { return m_uid1 == uid1 && m_uid2 == uid2; }

    //! Set device unique id
    void SetUID(uint32_t uid1,uint32_t uid2);

    //! Get the servo configuration as JSON
    virtual void ConfigAsJSON(Json::Value &value) const;

    //! Configure from JSON
    virtual bool ConfigureFromJSON(DogBotAPIC &api,const Json::Value &value);

    //! Update coms device
    virtual void UpdateComs(const std::shared_ptr<ComsC> &coms);

    //! Handle an announce packet
    bool HandlePacketAnnounce(const PacketDeviceIdC &pkt,bool isManager);

    //! Handle parameter update.
    //! Returns true if a value has changed.
    virtual bool HandlePacketReportParam(const PacketParam8ByteC &pkt,int size);

    //! Handle pong packet
    virtual void HandlePacketPong(const struct PacketPingPongC *pkt);

    //! Tick from main loop
    //! Used to check for communication timeouts.
    //! Returns true if state changed.
    virtual bool UpdateTick(TimePointT timeNow);

    //! Query setup information from the controller again.
    virtual void QueryRefresh();

    //! Restore configuration from stored settings.
    virtual bool RestoreConfig();

    //! Access notes.
    std::string Notes() const;

    //! Access serial number if set.
    std::string SerialNumber() const;

    //! Set notes.
    void SetNotes(const std::string &notes);

  protected:
    void Init();

    CallbackArrayC<ParameterUpdateFuncT> m_parameterCallbacks;

    mutable std::mutex m_mutexState;

    std::string m_deviceName;
    std::string m_notes;
    std::string m_serialNumber;

    std::shared_ptr<ComsC> m_coms;
    std::shared_ptr<spdlog::logger> m_log = spdlog::get("console");

    uint32_t m_uid1 = 0;
    uint32_t m_uid2 = 0;
    bool m_enabled = true;
    int m_id = -1; // Device id.
    ControlStateT m_controlState = CS_StartUp;
    TimePointT m_timeOfLastComs;
    bool m_online = false;

  };

}


#endif
