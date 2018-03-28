#ifndef DOGBOG_RELATIVEJOINT_HEADER
#define DOGBOG_RELATIVEJOINT_HEADER 1

#include "dogbot/Joint.hh"
#include <memory>

namespace DogBotN {

  //! Relative position joint.
  //! These give a simplified view of the robot.

  class JointRelativeC
   : public JointC
  {
  public:
    typedef std::chrono::time_point<std::chrono::steady_clock,std::chrono::duration< double > > TimePointT;

    JointRelativeC();

    //! Constructor
    JointRelativeC(std::shared_ptr<JointC> &jointDrive,std::shared_ptr<JointC> &jointRef);

    //! Destructor
    ~JointRelativeC();

    //! Type of joint
    virtual std::string JointType() const override;

    //! Configure from JSON
    virtual bool ConfigureFromJSON(DogBotAPIC &api,const Json::Value &value) override;

    //! Get the servo configuration as JSON
    virtual Json::Value ConfigAsJSON() const override;

    //! Get last reported state of the servo and the time it was taken.
    virtual bool GetState(TimePointT &tick,double &position,double &velocity,double &torque) const override;

    //! Estimate state at the given time.
    //! This will linearly extrapolate position, and assume velocity and torque are
    //! the same as the last reading.
    //! If the data is more than 5 ticks away from the
    virtual bool GetStateAt(TimePointT theTime,double &position,double &velocity,double &torque) const override;

    //! Update torque for the servo.
    virtual bool DemandTorque(float torque) override;

    //! Demand a position for the servo
    virtual bool DemandPosition(float position,float torqueLimit) override;

    //! Add a update callback for motor position
    virtual CallbackHandleC AddPositionUpdateCallback(const PositionUpdateFuncT &callback);

  protected:
    virtual bool Raw2Simple(
        float refPosition,float refVelocity,float refTorque,
        float drivePosition,float driveVelocity,float driveTorque,
        double &position,double &velocity,double &torque
    ) const;

    virtual bool Simple2Raw(
         float refPosition,float refTorque,
         float position,float torque,
         double &drivePosition,double &driveTorque
    ) const;


    CallbackHandleC m_driveCallback;

    float m_refGain = 1.0;
    float m_refOffset = 0.0;

    std::shared_ptr<JointC> m_jointDrive;  //!< Joint we're driving
    std::shared_ptr<JointC> m_jointRef;    //!< Joint we're using for the reference position

    float m_demandPosition = 0;
    float m_demandTorqueLimit = 0;
  };

}

#endif
