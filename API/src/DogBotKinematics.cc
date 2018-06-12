
#include "dogbot/DogBotKinematics.hh"
#include <fstream>

namespace DogBotN {


  DogBotKinematicsC::DogBotKinematicsC()
  {

  }

  //! Load a configuration file
  bool DogBotKinematicsC::LoadConfig(const std::string &configFile)
  {
    bool ret = true;
    if(configFile.empty())
      return false;
#if 0
    try {
#endif
      std::ifstream confStrm(configFile,std::ifstream::binary);

      if(!confStrm) {
        m_log->error("Failed to open configuration file '{}' ",configFile);
        return false;
      }

      Json::Value rootConfig;
      confStrm >> rootConfig;

      ret = ConfigureFromJSON(rootConfig);

#if 0
    } catch(std::exception &ex) {
      m_log->error("Failed to load configuration file. : {}", ex.what());
      return false;
    }
#endif
    return ret;
  }


  //! Configure from JSON
  bool DogBotKinematicsC::ConfigureFromJSON(const Json::Value &rootConfig)
  {
    Json::Value kinematicsList = rootConfig["kinematics"];
    if(kinematicsList.isNull()) {
      return false;
    }
    for(int i = 0;i < kinematicsList.size();i++) {
      Json::Value kinConf = kinematicsList[i];
      std::string name = kinConf.get("name","default").asString();
      std::shared_ptr<LegKinematicsC> kin;

      std::lock_guard<std::mutex> lock(m_mutexKinematics);
      for(auto &a : m_legKinematics) {
        if(a->Name() == name) {
          kin = a;
          break;
        }
      }
      if(!kin) {
        kin = std::make_shared<LegKinematicsC>(kinConf);
        m_legKinematics.push_back(kin);
      } else {
        kin->ConfigureFromJSON(kinConf);
      }
    }

    return true;
  }

  //! Get the servo configuration as JSON
  void DogBotKinematicsC::ConfigAsJSON(Json::Value &value) const
  {
    Json::Value kinematicsList;
    int index = 0;
    for(auto &a : m_legKinematics) {
      if(!a)
        continue;
      a->ConfigAsJSON();
      kinematicsList[index++] = a->ConfigAsJSON();
    }

    kinematicsList["kinematics"] = kinematicsList;
  }


  //! Get kinematics for leg by name
  std::shared_ptr<LegKinematicsC> DogBotKinematicsC::LegKinematicsByName(const std::string &name)
  {
    std::lock_guard<std::mutex> lock(m_mutexKinematics);
    for(auto &a : m_legKinematics) {
      if(a && a->Name() == name)
        return a;
    }
    return std::shared_ptr<LegKinematicsC>();
  }


}

