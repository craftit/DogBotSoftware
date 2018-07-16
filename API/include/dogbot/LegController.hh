#ifndef DOGBOG_LEGCONTROLLER_HEADER
#define DOGBOG_LEGCONTROLLER_HEADER 1

#include "dogbot/DogBotAPI.hh"
#include <memory>
#include "dogbot/LegKinematics.hh"
#include <eigen3/Eigen/Geometry>

namespace DogBotN {

  //! Class to manage the positioning of a single leg.

  class LegControllerC
  {
  public:
    LegControllerC()
    {}

    LegControllerC(std::shared_ptr<DogBotAPIC> &api,const std::string &legName,bool useVirtualKnee = false);

    //! Destructor
    virtual ~LegControllerC();

    //! Initialise controller
    virtual bool Init();

    //! Goto a position in the leg coordinate frame
    //! Returns true if the requested position is reachable
    bool Goto(float x,float y,float z,float torqueLimit);

    //! Goto a position in the leg coordinate frame
    //! Returns true if the requested position is reachable
    virtual bool Goto(const Eigen::Vector3f &at,float torqueLimit);

    //! Goto a joint angles
    virtual bool GotoJointAngles(const Eigen::Vector3f &angles,float torque);

    //! Get current joint angles
    // The vector has the angles  indexed in the following order : roll,pitch,knee
    virtual bool GetJointAngles(TimePointT theTime,Eigen::Vector3f &angles);

    //! Compute an estimate of the force on a foot and where it is.
    bool ComputeFootForce(const DogBotN::TimePointT &atTime,Eigen::Vector3f &position,Eigen::Vector3f &force);

    //! Access leg kinematics.
    DogBotN::LegKinematicsC &Kinematics()
    { return *m_kinematics; }

  protected:
    bool m_useVirtualKnee = true;
    std::string m_legName;
    std::shared_ptr<spdlog::logger> m_log = spdlog::get("console");
    std::shared_ptr<DogBotN::LegKinematicsC> m_kinematics;
    std::shared_ptr<DogBotAPIC> m_api;
    std::vector<std::string> m_legJointNames;
    std::shared_ptr<JointC> m_joints[3];
    float m_torqueLimit = 2.0;

    Eigen::Vector3f m_legOrigin;
  };

}

#endif
