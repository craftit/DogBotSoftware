#ifndef DOGBOG_DEVICEIMU_HEADER
#define DOGBOG_DEVICEIMU_HEADER 1

#include "dogbot/Device.hh"
#include <eigen3/Eigen/Geometry>

namespace DogBotN
{

  //! Single frame of IMU data.

  class IMUFrameC
  {
  public:
    IMUFrameC()
      : m_acceleration(0,0,0),
        m_rotationVelocity(0,0,0),
        m_orientation(1,0,0,0)
    {}

    IMUFrameC(
        const Eigen::Vector3f &acceleration,
        const Eigen::Vector3f &rotationVelocity,
        const Eigen::Quaternionf &orientation,
        float groundDistance
        )
     : m_acceleration(acceleration),
       m_rotationVelocity(rotationVelocity),
       m_orientation(orientation),
       m_groundDistance(groundDistance)
    {}

    //! Position
    const Eigen::Vector3f &Acceleration() const
    { return m_acceleration; }

    //! Rotation
    const Eigen::Vector3f RotationVelocity() const
    { return m_rotationVelocity; }

    //! Orientation
    const Eigen::Quaternionf &Orientaton() const
    { return m_orientation; }

    //! Access ground distance
    float GroundDistance() const
    { return m_groundDistance; }

  protected:
    Eigen::Vector3f m_acceleration;
    Eigen::Vector3f m_rotationVelocity;
    Eigen::Quaternionf m_orientation;
    float m_groundDistance = -1.0;
  };


  //! Interface to IMU

  class DeviceIMUC
   : public DeviceC
  {
  public:
    typedef std::function<void (const TimePointT &when,const IMUFrameC &frame)> IMUUpdateFuncT;
    typedef std::function<void (const TimePointT &when,int sensorId,float distance)> RangeUpdateFuncT;

    // Construct from coms link and deviceId
    DeviceIMUC(const std::shared_ptr<ComsC> &coms,int deviceId);

    // Construct with announce packet.
    DeviceIMUC(const std::shared_ptr<ComsC> &coms,int deviceId,const PacketDeviceIdC &pktAnnounce);

    //! Get the servo configuration as JSON
    virtual void ConfigAsJSON(Json::Value &value) const;

    //! Configure from JSON
    virtual bool ConfigureFromJSON(DogBotAPIC &api,const Json::Value &value);

    //! Access the device type
    virtual const char *DeviceType() const;

    //! Handle an incoming IMU packet.
    bool HandlePacketIMU(struct PacketIMUC *imu);

    //! Handle an incoming range packet.
    bool HandlePacketRange(struct PacketRangeC *imu);

    //! Get last received IMU state.
    //! when - Time at which data was received.
    //! frame - IMU information
    void GetState(TimePointT &when,IMUFrameC &frame);

    //! Add a update callback for imu reading
    CallbackHandleC AddIMUCallback(const IMUUpdateFuncT &callback)
    { return m_imuCallbacks.Add(callback); }

    //! Add a update callback for range reading
    CallbackHandleC AddRangeCallback(const RangeUpdateFuncT &callback)
    { return m_rangeCallbacks.Add(callback); }

    //! Access the last update time.
    TimePointT LastUpdateTime() const
    { return m_whenIMU; }

  protected:

    CallbackArrayC<IMUUpdateFuncT > m_imuCallbacks;
    CallbackArrayC<RangeUpdateFuncT > m_rangeCallbacks;

    TimePointT m_whenIMU;
    IMUFrameC m_frame;

    TimePointT m_whenRange;
    float m_groundDistance = 0;
    float m_rangeOffset = 0.01;

  };

}

#endif
