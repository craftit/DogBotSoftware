
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
    Eigen::Quaternionf rot(pkt->m_rot[0]*scale,pkt->m_rot[1]*scale,pkt->m_rot[2]*scale,pkt->m_rot[3]*scale);
    rot.normalize();

    float groundDistance  = ((float) pkt->m_groundDistance) ;

    IMUFrameC frame(acceleration,rotationVelocity,rot,groundDistance);
    {
      std::lock_guard<std::mutex> lock(m_mutexState);
      m_when = when;
      m_frame = frame;
    }

    //m_log->info("{} : Accel {} {} {} Gyro {} {} {} Orientation {} {} {} {} ",DeviceName(),pkt->m_accel[0],pkt->m_accel[1],pkt->m_accel[2],pkt->m_gyro[0],pkt->m_gyro[1],pkt->m_gyro[2],pkt->m_rot[0],pkt->m_rot[1],pkt->m_rot[2],pkt->m_rot[3]);
    m_log->info("{} : Distance {}   Rot: {} {} {} {} ",DeviceName(),pkt->m_groundDistance,rot.w(),rot.x(),rot.y(),rot.z());
    for(auto &a : m_imuCallbacks.Calls()) {
      if(a) a(when,frame);
    }

    return true;
  }

  //! Get last received IMU state.
  void DeviceIMUC::GetState(TimePointT &when,IMUFrameC &frame)
  {
    std::lock_guard<std::mutex> lock(m_mutexState);
    when = m_when;
    frame = m_frame;
  }


}

