#ifndef DOGBOG_DEVICEPLATFORMMANAGER_HEADER
#define DOGBOG_DEVICEPLATFORMMANAGER_HEADER 1

#include "dogbot/Device.hh"

namespace DogBotN
{
  //! Interface to IMU

  class DevicePlatformManagerC
   : public DeviceC
  {
  public:

    // Construct from coms link and deviceId
    DevicePlatformManagerC(const std::shared_ptr<ComsC> &coms,int deviceId);

    // Construct with announce packet.
    DevicePlatformManagerC(const std::shared_ptr<ComsC> &coms,int deviceId,const PacketDeviceIdC &pktAnnounce);

    //! Get the servo configuration as JSON
    virtual void ConfigAsJSON(Json::Value &value) const;

    //! Configure from JSON
    virtual bool ConfigureFromJSON(DogBotAPIC &api,const Json::Value &value);

    //! Access the device type
    virtual const char *DeviceType() const;

    //! Start activity, homing
    bool StartActivity(enum PlatformActivityT pa);

    //! Stop current activity started by this interface.
    bool StopActivity();

    //! Stop any activity
    bool AbortActivity();


  protected:
    uint32_t m_activityKey = 0;
  };

}

#endif
