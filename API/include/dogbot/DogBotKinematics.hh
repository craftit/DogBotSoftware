#ifndef DOGBOG_DOGBOTKINEMATICS_HEADER
#define DOGBOG_DOGBOTKINEMATICS_HEADER 1

#include "dogbot/LegKinematics.hh"
#include <mutex>
#include <memory>
#include <spdlog/spdlog.h>
#include "dogbot/SimpleQuadrupedPose.hh"
#include "dogbot/PoseAngles.hh"

namespace DogBotN {

  //! DogBot kinematics information

  class DogBotKinematicsC
  {
  public:
    DogBotKinematicsC();

    //! Load a configuration file
    bool LoadConfig(const std::string &configFile);

    //! Configure from JSON
    bool ConfigureFromJSON(const Json::Value &value);

    //! Get the servo configuration as JSON
    void ConfigAsJSON(Json::Value &value) const;

    //! Get kinematics for leg by name
    std::shared_ptr<LegKinematicsC> LegKinematicsByName(const std::string &name);

    //! Get kinematics for leg by name
    const std::shared_ptr<LegKinematicsC> &LegKinematicsByNumber(int i) const
    {
      assert(i >= 0 && i < m_legKinematicsByNumber.size());
      assert(m_legKinematicsByNumber[i]);
      return m_legKinematicsByNumber[i];
    }

    //! Convert a set of foot positions to joint angles.
    //! Returns true if all positions are reachable. false otherwise.
    bool Pose2Angles(const SimpleQuadrupedPoseC &pose,PoseAnglesC &poseAngles,bool useVirtualJoints = false) const;

    //! Access an ordered list of leg names
    static const std::vector<std::string> &LegNames();

    //! Access names of leg joints.
    static const std::vector<std::string> &LegJointNames();

    //! Compute the maximum extension of the legs
    // This is the maximum extension all the legs are capable of
    float MaxLegExtension() const;

    //! Compute the minimum extension of the legs
    // This is the minimum extension all the legs are capable of
    float MinLegExtension() const;

    //! Compute the maximum stride length at a given leg extension.
    float StrideLength(float extension) const;
  protected:
    std::shared_ptr<spdlog::logger> m_log = spdlog::get("console");
    std::mutex m_mutexKinematics;
    std::vector<std::shared_ptr<LegKinematicsC> > m_legKinematics;
    std::vector<std::shared_ptr<LegKinematicsC> > m_legKinematicsByNumber;
  };

}


#endif
