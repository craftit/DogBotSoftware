
#define _BSD_SOURCE 1

#include "dogbot/Device.hh"
#include "dogbot/Util.hh"
#include <string>
#include <cmath>

namespace DogBotN {

  DeviceC::DeviceC()
  {}

  // Construct from coms link and deviceId
  DeviceC::DeviceC(const std::shared_ptr<ComsC> &coms,int deviceId)
  : m_id(deviceId),
    m_coms(coms)
  {
    if(deviceId != 0)
      m_deviceName = std::to_string(deviceId);
    Init();
  }

  // Construct with announce packet.
  DeviceC::DeviceC(const std::shared_ptr<ComsC> &coms,int deviceId,const PacketDeviceIdC &pktAnnounce)
  : m_uid1(pktAnnounce.m_uid[0]),
    m_uid2(pktAnnounce.m_uid[1]),
    m_id(deviceId),
    m_coms(coms)
  {
    m_serialNumber =  std::to_string(pktAnnounce.m_uid[0]) + "-" + std::to_string(pktAnnounce.m_uid[1]);
    m_deviceName = m_serialNumber;
    Init();
  }


  //! Destructor
  DeviceC::~DeviceC()
  {}

  void DeviceC::Init()
  {

  }

  //! Access the device type
  const char *DeviceC::DeviceType() const
  { return "device"; }

  //! Access serial number if set.
  std::string DeviceC::SerialNumber() const
  {
    std::lock_guard<std::mutex> lock(m_mutexState);
    return m_serialNumber;
  }

  //! Access notes.
  std::string DeviceC::Notes() const
  {
    std::lock_guard<std::mutex> lock(m_mutexState);
    return m_notes;
  }

  //! Set notes.
  void DeviceC::SetNotes(const std::string &notes)
  {
    std::lock_guard<std::mutex> lock(m_mutexState);
    m_notes = notes;
  }

  //! Set device unique id
  void DeviceC::SetUID(uint32_t uid1,uint32_t uid2)
  {
    m_uid1 = uid1;
    m_uid2 = uid2;
  }

  //! Get the servo configuration as JSON
  void DeviceC::ConfigAsJSON(Json::Value &ret) const
  {
    std::lock_guard<std::mutex> lock(m_mutexState);
    ret["device_name"] = m_deviceName;
    ret["notes"] = m_notes;
    ret["serial_number"] = m_serialNumber;
    ret["device_type"] = DeviceType();

    ret["uid1"] = m_uid1;
    ret["uid2"] = m_uid2;
    ret["deviceId"] = m_id;

  }

  //! Configure from JSON
  bool DeviceC::ConfigureFromJSON(DogBotAPIC &api,const Json::Value &conf)
  {
    std::lock_guard<std::mutex> lock(m_mutexState);
    m_deviceName = conf.get("device_name",conf.get("name","?").asString()).asString();
    m_notes = conf.get("notes","").asString();
    m_serialNumber = conf.get("serial_number",m_serialNumber).asString();
    m_uid1 = conf.get("uid1",0u).asUInt();
    m_uid2 = conf.get("uid2",0u).asUInt();
    m_enabled = conf.get("enabled",false).asBool();
    return true;
  }

  //! Update coms device
  void DeviceC::UpdateComs(const std::shared_ptr<ComsC> &coms)
  {
    m_coms = coms;
  }


  //! Handle an incoming announce message.
  bool DeviceC::HandlePacketAnnounce(const PacketDeviceIdC &pkt,bool isManager)
  {
    bool ret = false;
    if(pkt.m_deviceId != m_id && isManager) {
      //m_log->info("Updating device {} {} with id {} ",m_uid1,m_uid2,m_id);
      m_coms->SendSetDeviceId(m_id,m_uid1,m_uid2);
      ret = true;
    }
    std::lock_guard<std::mutex> lock(m_mutexState);
    auto timeNow = std::chrono::steady_clock::now();
    m_timeOfLastComs = timeNow;
    return ret;
  }

  //! Handle pong packet
  void DeviceC::HandlePacketPong(const struct PacketPingPongC *pkt)
  {
    std::lock_guard<std::mutex> lock(m_mutexState);
    auto timeNow = std::chrono::steady_clock::now();
    m_timeOfLastComs = timeNow;
  }

  //! Handle parameter update.
  //! Returns true if a value has changed.
  bool DeviceC::HandlePacketReportParam(const PacketParam8ByteC &pkt,int size)
  {
    std::lock_guard<std::mutex> lock(m_mutexState);
    auto timeNow = std::chrono::steady_clock::now();
    m_timeOfLastComs = timeNow;
    return true;
  }

  //! Tick from main loop
  //! Used to check for communication timeouts.
  //! Returns true if state changed.
  bool DeviceC::UpdateTick(TimePointT timeNow)
  {
    return true;
  }

  //! Query setup information from the controller again.
  void DeviceC::QueryRefresh()
  {
  }

  //! Restore configuration from stored settings.
  bool DeviceC::RestoreConfig()
  {
    return true;
  }

}
