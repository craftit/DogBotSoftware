#ifndef DOGBOT_DOGBOTCONTROLLER_HEADER
#define DOGBOT_DOGBOTCONTROLLER_HEADER 1

#include "dogbot/DogBotAPI.hh"
#include <memory>
#include "dogbot/PoseAngles.hh"

namespace DogBotN {

  //! Class to manage the position of all the leg joints on the robot.

  class DogBotControllerC
  {
  public:
    DogBotControllerC()
    {}

    DogBotControllerC(std::shared_ptr<DogBotAPIC> &api);

    //! Destructor
    virtual ~DogBotControllerC();

    //! Setup trajectory
    virtual bool SetupTrajectory(float updatePeriod,float torqueLimit);

    //! Send next trajectory position, this should be called at 'updatePeriod' intervals as setup
    //! with SetupTrajectory.
    virtual bool NextTrajectory(const PoseAnglesC &pose);

    //! Compute the joint angles from pose information
    // If the angle computation fails, this will use the closest reachable point to one requested.
    // Returns false if the updates succeeded for all servos.
    virtual bool NextTrajectory(const SimpleQuadrupedPoseC &pose);

  protected:
    //! Initialise controller
    virtual bool Init();

    std::string m_legName;
    std::shared_ptr<spdlog::logger> m_log = spdlog::get("console");
    std::shared_ptr<DogBotAPIC> m_api;
    std::shared_ptr<JointC> m_joints[12];
    float m_torqueLimit = 2.0;
  };

}

#endif
