#ifndef DOGBOG_LEGCONTROLLER_HEADER
#define DOGBOG_LEGCONTROLLER_HEADER 1

#include "dogbot/DogBotAPI.hh"
#include <memory>
#include "dogbot/LegKinematics.hh"

namespace DogBotN {

  //! Class to manage the positioning of a single leg.

  class LegControllerC
  {
  public:
    LegControllerC(std::shared_ptr<DogBotAPIC> &api,const std::string &legName,bool useVirtualKnee = false);

    //! Destructor
    virtual ~LegControllerC();

    //! Initialise controller
    virtual bool Init();

    //! Goto a position in the leg coordinate frame
    //! Returns true if the requested position is reachable
    virtual bool Goto(float x,float y,float z,float torque);

  protected:
    bool m_useVirtualKnee = true;
    std::string m_legName;
    std::shared_ptr<spdlog::logger> m_log = spdlog::get("console");
    std::shared_ptr<DogBotN::LegKinematicsC> m_kinematics;
    std::shared_ptr<DogBotAPIC> m_api;
    std::vector<std::string> m_legJointNames;
    std::shared_ptr<JointC> m_joints[3];
    float m_torqueLimit = 2.0;
  };

}

#endif
