
#include "dogbot/DeviceIMU.hh"


namespace DogBotN
{

  // Construct from coms link and deviceId
  DeviceIMUC::DeviceIMUC(const std::shared_ptr<ComsC> &coms,int deviceId)
   : DeviceC(coms,deviceId)
  {
  }

  // Construct with announce packet.
  DeviceIMUC::DeviceIMUC(const std::shared_ptr<ComsC> &coms,int deviceId,const PacketDeviceIdC &pktAnnounce)
   : DeviceC(coms,deviceId,pktAnnounce)
  {
  }

  //! Access the device type
  const char *DeviceIMUC::DeviceType() const
  { return "imu"; }

  //! Handle an incoming IMU packet.
  bool DeviceIMUC::HandlePacketIMU(struct PacketIMUC *pkt)
  {
    m_log->info("{} : Accel {} {} {} Gyro {} {} {} ",DeviceName(),pkt->m_accel[0],pkt->m_accel[1],pkt->m_accel[2],pkt->m_gyro[0],pkt->m_gyro[1],pkt->m_gyro[2]);
    return true;
  }


}

