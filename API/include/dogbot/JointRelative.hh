#ifndef DOGBOG_RELATIVEJOINT_HEADER
#define DOGBOG_RELATIVEJOINT_HEADER 1

#include "dogbot/Joint.hh"
#include <memory>

namespace DogBotN {

  //! Abstract joint.
  //! These give a simplified view of the robot.

  class JointRelativeC
   : public JointC
  {
  public:
    typedef std::chrono::time_point<std::chrono::steady_clock,std::chrono::duration< double > > TimePointT;

    JointRelativeC();

    //! Constructor
    JointRelativeC(std::shared_ptr<JointC> &jointDrive,std::shared_ptr<JointC> &jointRef);

    //! Configure from JSON
    virtual bool ConfigureFromJSON(DogBotAPIC &api,const Json::Value &value) override;

    //! Get the servo configuration as JSON
    virtual Json::Value ConfigAsJSON() const override;

    //! Get last reported state of the servo and the time it was taken.
    virtual bool GetState(TimePointT &tick,float &position,float &velocity,float &torque) const override;

    //! Estimate state at the given time.
    //! This will linearly extrapolate position, and assume velocity and torque are
    //! the same as the last reading.
    //! If the data is more than 5 ticks away from the
    virtual bool GetStateAt(TimePointT theTime,float &position,float &velocity,float &torque) const override;

    //! Update torque for the servo.
    virtual bool DemandTorque(float torque) override;

    //! Demand a position for the servo
    virtual bool DemandPosition(float position,float torqueLimit) override;

  protected:
    virtual bool Raw2Simple(
        float refPosition,float refVelocity,float refTorque,
        float drivePosition,float driveVelocity,float driveTorque,
        float &position,float &velocity,float &torque
    ) const;

    virtual bool Simple2Raw(
         float refPosition,float refTorque,
         float position,float torque,
         float &drivePosition,float &driveTorque
    ) const;


    float m_refGain = 1.0;
    float m_refOffset = 0.0;

    std::shared_ptr<JointC> m_jointDrive;  //!< Joint we're driving
    std::shared_ptr<JointC> m_jointRef;    //!< Joint we're using for the reference position

    float m_demandPosition = 0;
    float m_demandTorqueLimit = 0;
  };

}

#endif
