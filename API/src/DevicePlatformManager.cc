
#include "dogbot/DevicePlatformManager.hh"
#include "dogbot/Strings.hh"
#include <time.h>

#define DODEBUG 0
#if DODEBUG
#define ONDEBUG(x) x
#else
#define ONDEBUG(x)
#endif

namespace DogBotN
{

  // Construct from coms link and deviceId
  DevicePlatformManagerC::DevicePlatformManagerC(const std::shared_ptr<ComsC> &coms,int deviceId)
   : DeviceC(coms,deviceId)
  {
    SetDeviceName("PlatformManager");
    ONDEBUG(m_log->info("DevicePlaformManager created {} ",deviceId));
  }

  // Construct with announce packet.
  DevicePlatformManagerC::DevicePlatformManagerC(const std::shared_ptr<ComsC> &coms,int deviceId,const PacketDeviceIdC &pktAnnounce)
   : DeviceC(coms,deviceId,pktAnnounce)
  {
    SetDeviceName("PlatformManager");
    ONDEBUG(m_log->info("DevicePlaformManager created {} ",deviceId));
  }

  //! Get the servo configuration as JSON
  void DevicePlatformManagerC::ConfigAsJSON(Json::Value &ret) const
  {
    DeviceC::ConfigAsJSON(ret);
  }

  //! Configure from JSON
  bool DevicePlatformManagerC::ConfigureFromJSON(DogBotAPIC &api,const Json::Value &value)
  {
    bool ret = DeviceC::ConfigureFromJSON(api,value);
    return ret;
  }

  //! Access the device type
  const char *DevicePlatformManagerC::DeviceType() const
  { return "PlatformManger"; }

  //! Start activity, homing
  bool DevicePlatformManagerC::StartActivity(enum PlatformActivityT pa)
  {
    if(m_activityKey == 0) {
      m_activityKey = getpid() + clock();
    }
    ONDEBUG(m_log->info("Sending start activity: {} ",DogBotN::ComsPlatformActivityToString(pa)));
    m_coms->SendSetPlaformActivity(m_id,m_activityKey,pa);

    return true;
  }

  //! Stop current activity
  bool DevicePlatformManagerC::StopActivity()
  {
    ONDEBUG(m_log->info("Sending stop activity"));
    m_coms->SendSetPlaformActivity(m_id,m_activityKey,PA_Idle);
    return true;
  }

  //! Stop any activity
  bool DevicePlatformManagerC::AbortActivity()
  {
    ONDEBUG(m_log->info("Sending abort activity"));
    m_coms->SendSetPlaformActivity(m_id,0,PA_Idle);
    return true;
  }



}

