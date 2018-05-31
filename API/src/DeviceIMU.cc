
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
    TimePointT when = TimePointT::clock::now();

    Eigen::Vector3f acceleration(pkt->m_accel[0]/100.0,pkt->m_accel[1]/100.0,pkt->m_accel[2]/100.0);
    Eigen::Vector3f rotationVelocity(pkt->m_gyro[0]/16.0,pkt->m_gyro[1]/16.0,pkt->m_gyro[2]/16.0);

    const double scale = (1.0 / (1<<14));
    Eigen::Quaternionf orientation(pkt->m_rot[0]*scale,pkt->m_rot[1]*scale,pkt->m_rot[2]*scale,pkt->m_rot[3]*scale);

    IMUFrameC frame(acceleration,rotationVelocity,orientation);

    //m_log->info("{} : Accel {} {} {} Gyro {} {} {} ",DeviceName(),pkt->m_accel[0],pkt->m_accel[1],pkt->m_accel[2],pkt->m_gyro[0],pkt->m_gyro[1],pkt->m_gyro[2]);
    //m_log->info("{} : Distance {} ",DeviceName(),pkt->m_groundDistance);
    for(auto &a : m_imuCallbacks.Calls()) {
      if(a) a(when,frame);
    }

    return true;
  }


}

