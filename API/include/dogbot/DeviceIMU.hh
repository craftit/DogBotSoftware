#ifndef DOGBOG_DEVICEIMU_HEADER
#define DOGBOG_DEVICEIMU_HEADER 1

#include "dogbot/Device.hh"

namespace DogBotN
{

  class DeviceIMUC
   : public DeviceC
  {
  public:
    // Construct from coms link and deviceId
    DeviceIMUC(const std::shared_ptr<ComsC> &coms,int deviceId);

    // Construct with announce packet.
    DeviceIMUC(const std::shared_ptr<ComsC> &coms,int deviceId,const PacketDeviceIdC &pktAnnounce);

    //! Access the device type
    virtual const char *DeviceType() const;

    //! Handle an incoming IMU packet.
    bool HandlePacketIMU(struct PacketIMUC *imu);

  protected:

  };

}

#endif
