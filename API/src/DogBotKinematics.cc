
#include "dogbot/DogBotKinematics.hh"
#include "dogbot/DogBotAPI.hh"
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

    m_legKinematicsByNumber.clear();
    for(int i = 0;i < 4;i++) {
      m_legKinematicsByNumber.push_back(LegKinematicsByName(DogBotAPIC::LegNames()[i]));
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

    value["kinematics"] = kinematicsList;
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

  //! Access an ordered list of leg names
  const std::vector<std::string> &DogBotKinematicsC::LegNames()
  {
    static std::vector<std::string> g_legNames = {"front_left","front_right","back_left","back_right"};
    return g_legNames;
  }

  //! Access names of leg joints.
  const std::vector<std::string> &DogBotKinematicsC::LegJointNames()
  {
    static std::vector<std::string> g_legJointNames = {"roll","pitch","knee"};
    return g_legJointNames;
  }

  //! Convert a set of foot positions to joint angles.
  //! Returns true if all positions are reachable. false otherwise.
  bool DogBotKinematicsC::Pose2Angles(
      const SimpleQuadrupedPoseC &pose,
      PoseAnglesC &poseAngles,
      bool useVirtualJoints
      ) const
  {
    bool ret = true;
    for(int i = 0;i < 4;i++) {
      assert(m_legKinematicsByNumber[i]);
      Eigen::Vector3f angles;
      if(useVirtualJoints) {
        if(!m_legKinematicsByNumber[i]->InverseVirtual(pose.FootPosition(i),angles))
          ret = false;
      } else {
        if(!m_legKinematicsByNumber[i]->InverseDirect(pose.FootPosition(i),angles))
          ret = false;
      }
      poseAngles.SetLegJointAngles(i,angles);
    }
    return ret;
  }


  //! Compute the maximum extension of the legs
  // This is the maximum extension all the legs are capable of
  float DogBotKinematicsC::MaxLegExtension() const
  {
    float maxExt = 0;
    auto it = m_legKinematics.begin();
    if(it != m_legKinematics.end()) {
      assert(*it);
      maxExt = (*it)->MaxExtension();
      for(;it != m_legKinematics.end();++it) {
        assert(*it);
        float ext = (*it)->MaxExtension();
        if(ext < maxExt) {
          maxExt = ext;
        }
      }
    }
    return maxExt;
  }

  //! Compute the minimum extension of the legs
  // This is the minimum extension all the legs are capable of
  float DogBotKinematicsC::MinLegExtension() const
  {
    float minExt = 0;
    auto it = m_legKinematics.begin();
    if(it != m_legKinematics.end()) {
      assert(*it);
      minExt = (*it)->MinExtension();
      for(;it != m_legKinematics.end();++it) {
        assert(*it);
        float ext = (*it)->MinExtension();
        if(ext > minExt) {
          minExt = ext;
        }
      }
    }
    return minExt;
  }

  //! Compute the maximum stride length at a given z position
  float DogBotKinematicsC::StrideLength(float zoffset) const
  {
    float maxStride = 0;
    auto it = m_legKinematics.begin();
    if(it != m_legKinematics.end()) {
      assert(*it);
      maxStride = (*it)->StrideLength(zoffset);
      for(;it != m_legKinematics.end();++it) {
        assert(*it);
        float ext = (*it)->StrideLength(zoffset);
        if(ext < maxStride) {
          maxStride = ext;
        }
      }
    }
    return maxStride;
  }




}

