#ifndef DOGBOG_JOINT4BARLINKAGE_HEADER
#define DOGBOG_JOINT4BARLINKAGE_HEADER 1

#include "dogbot/JointRelative.hh"
#include <memory>
#include "dogbot/LegKinematics.hh"

namespace DogBotN {

  //! Virtual joint.
  //! These give a simplified view of the robot.

  class Joint4BarLinkageC
   : public JointRelativeC
  {
  public:
    typedef std::chrono::time_point<std::chrono::steady_clock,std::chrono::duration< double > > TimePointT;

    Joint4BarLinkageC();

    //! Constructor
    Joint4BarLinkageC(std::shared_ptr<JointC> &jointDrive,std::shared_ptr<JointC> &jointRef);

    //! Type of joint
    virtual std::string JointType() const override;

    //! Configure from JSON
    virtual bool ConfigureFromJSON(DogBotAPIC &api,const Json::Value &value) override;

    //! Get the servo configuration as JSON
    virtual void ConfigAsJSON(Json::Value &value) const override;

    //! Access smallest angle allowed by joint.
    float MinAngle() const
    { return m_minAngle; }

    //! Access largest angle allowed by joint.
    float MaxAngle() const
    { return m_maxAngle; }

  protected:
    //! Initialise joint
    void Init();

    virtual bool Raw2Simple(
        float refPosition,float refVelocity,float refTorque,
        float drivePosition,float driveVelocity,float driveTorque,
        double &position,double &velocity,double &torque
    ) const override;

    virtual bool Simple2Raw(
         float refPosition,float refTorque,
         float position,float torque,
         double &drivePosition,double &driveTorque
         ) const override;

    float m_minAngle = -M_PI*2;
    float m_maxAngle = M_PI*2;
    std::shared_ptr<LegKinematicsC> m_legKinematics; //!< Kinematics for linkage
  };

}

#endif
