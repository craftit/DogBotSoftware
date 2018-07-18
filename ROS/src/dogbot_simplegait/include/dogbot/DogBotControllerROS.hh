#ifndef DOGBOT_DOGBOTCONTROLLER_HEADER
#define DOGBOT_DOGBOTCONTROLLER_HEADER 1

#include <memory>
#include "dogbot/PoseAngles.hh"
#include "dogbot/SimpleQuadrupedPose.hh"
#include "dogbot/DogBotKinematics.hh"

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

namespace DogBotN {

  //! Class to manage the position of all the leg joints on the robot.

  class DogBotControllerROSC
  {
  public:
    DogBotControllerROSC();

    //! Setup controller with kinematics
    DogBotControllerROSC(
        const std::string &name,
        const std::shared_ptr<DogBotKinematicsC> &dogBotKinematics
        );


    //! Destructor
    virtual ~DogBotControllerROSC();

    //! Setup trajectory
    virtual bool SetupTrajectory(float updatePeriod,float torqueLimit);

    //! Send next trajectory position, this should be called at 'updatePeriod' intervals as setup
    //! with SetupTrajectory.
    virtual bool NextTrajectory(const PoseAnglesC &pose);

    //! Compute the joint angles from pose information
    // If the angle computation fails, this will use the closest reachable point to one requested.
    // Returns false if the updates succeeded for all servos.
    virtual bool NextTrajectory(const SimpleQuadrupedPoseC &pose);

    //! Generate virtual joint positions ?
    //! Enabled by default.
    void SetUseVirtualJoints(bool useVirtual)
    { m_useVirtualJoints = useVirtual; }

  protected:
    void Init();

    std::shared_ptr<DogBotKinematicsC> m_dogBotKinematics;

    std::string m_robotNamespace;

    ros::NodeHandle m_node;

    std::vector<float> m_polarity;
    std::vector<ros::Publisher> m_jointControllers;

    bool m_useVirtualJoints = true;
  };

}

#endif
