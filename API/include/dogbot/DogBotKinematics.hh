#ifndef DOGBOG_DOGBOTKINEMATICS_HEADER
#define DOGBOG_DOGBOTKINEMATICS_HEADER 1

#include "dogbot/LegKinematics.hh"
#include <mutex>
#include <memory>
#include <spdlog/spdlog.h>

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
      assert(i >= 0 && i < 4);
      assert(m_legKinematicsByNumber[i]);
      return m_legKinematicsByNumber[i];
    }

  protected:
    std::shared_ptr<spdlog::logger> m_log = spdlog::get("console");
    std::mutex m_mutexKinematics;
    std::vector<std::shared_ptr<LegKinematicsC> > m_legKinematics;
    std::vector<std::shared_ptr<LegKinematicsC> > m_legKinematicsByNumber;
  };

}


#endif
