

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <getopt.h>

#include "dogbot/DogBotAPI.hh"
#include "dogbot/Util.hh"
#include "dogbot/LegController.hh"
#include "cxxopts.hpp"

// This provides a network interface for controlling the servos via ZMQ.

int main(int argc,char **argv)
{
  std::string devFilename = "local";
  std::string configFile = DogBotN::DogBotAPIC::DefaultConfigFile();
  std::string jointName = "front_right_knee";
  std::string firmwareFile;
  std::string loadPoseFile;

  auto logger = spdlog::stdout_logger_mt("console");
  bool dumpPose = false;
  bool cycle = false;
  int jointId = -1;
  float torque = 1.0;
  float range = 45.0;
  float angle = 0;
  int delay = 5000;
  float speed  = -1.0;
  bool useVirtualJoint = false;
  try
  {
    cxxopts::Options options(argv[0], "DogBot hardware manager");
    options
      .positional_help("[optional args]")
      .show_positional_help();

    options.add_options()
      ("c,config", "Configuration file", cxxopts::value<std::string>(configFile))
      ("d,device", "Device to use from communication. Typically 'local' for local server or 'usb' for direct connection ", cxxopts::value<std::string>(devFilename))
      ("i,target","Controller id to use", cxxopts::value<int>(jointId))
      ("t,torque","Torque to use", cxxopts::value<float>(torque))
      ("j,joint","Joint name", cxxopts::value<std::string>(jointName))
      ("r,range","Motion range", cxxopts::value<float>(range))
      ("a,angle","Centre angle", cxxopts::value<float>(angle))
      ("p,pose","Joint name", cxxopts::value<bool>(dumpPose))
      ("l,load","Load pose", cxxopts::value<std::string>(loadPoseFile))
      ("s,speed","Speed for smooth movement", cxxopts::value<float>(speed))
      ("g,cycle","Cycle up and down",cxxopts::value<bool>(cycle))
      ("v,virtual-joint","Use virtual joint",cxxopts::value<bool>(useVirtualJoint))
      ("o,delay","Delay in cycle, default is 5000",cxxopts::value<int>(delay))
      ("h,help", "Print help")
    ;

    auto result = options.parse(argc, argv);

    if (result.count("help"))
    {
      std::cout << options.help({""}) << std::endl;
      exit(0);
    }

  } catch (const cxxopts::OptionException& e)
  {
    std::cout << "error parsing options: " << e.what() << std::endl;
    exit(1);
  }


  logger->info("Dumping leg coordinates ");
  logger->info("Using config file: '{}'",configFile);
  logger->info("Using communication type: '{}'",devFilename);

  std::shared_ptr<DogBotN::DogBotAPIC> dogbot = std::make_shared<DogBotN::DogBotAPIC>(
      devFilename,
      configFile,
      logger
      );

  // Wait for poses to update and things to settle.
  sleep(3);

  if(dumpPose) {
    std::vector<std::shared_ptr<DogBotN::ServoC> > list = dogbot->ListServos();

    Json::Value poseJSON;
    for(auto &a : list) {
      if(!a || a->Name() == "spare")
        continue;
      poseJSON[a->Name()] =  DogBotN::Rad2Deg(a->Position());
    }
    std::cout << poseJSON << std::endl;
    return 1;
  }

  if(!loadPoseFile.empty()) {

    std::ifstream inFile(loadPoseFile);
    if(!inFile) {
      logger->error("Failed to load {} ",loadPoseFile);
      return 1;
    }
    Json::Value poseJSON;
    inFile >> poseJSON;
    std::vector<std::shared_ptr<DogBotN::JointC> > jnts;
    std::vector<float> position;
    for(auto iter = poseJSON.begin();iter != poseJSON.end();++iter) {
      std::shared_ptr<DogBotN::JointC> jnt = dogbot->GetJointByName(iter.key().asString());
      if(!jnt) {
        logger->error("Failed to find joint {} ",iter.key().asString());
        return 1;
      }
      jnts.push_back(jnt);
      position.push_back(iter->asFloat());
    }

    for(int i = 0;i < jnts.size();i++) {
      jnts[i]->DemandPosition(DogBotN::Deg2Rad(position[i]),torque);
    }
    logger->info("Move done.");
    return 0;
  }

  if(cycle) {

    // Lift the speed limit a bit
    dogbot->Connection()->SetParam(0,CPI_VelocityLimit,(float) 300.0);

    std::shared_ptr<DogBotN::LegControllerC> legs[4];
    legs[3] = std::make_shared<DogBotN::LegControllerC>(dogbot,"front_right",useVirtualJoint);
    legs[2] = std::make_shared<DogBotN::LegControllerC>(dogbot,"front_left",useVirtualJoint);
    legs[1] = std::make_shared<DogBotN::LegControllerC>(dogbot,"back_right",useVirtualJoint);
    legs[0] = std::make_shared<DogBotN::LegControllerC>(dogbot,"back_left",useVirtualJoint);

    while(1) {
      for(int i = 0;i < 360;i++) {
        // 0.35 to 0.7
        float z = cos(DogBotN::Deg2Rad(i)) * 0.17 +0.525;

        for(int i = 0;i < 4;i++) {
          legs[i]->Goto(Eigen::Vector3f(0,0,-z),torque);
        }
        usleep(delay);
      }
    }

  }

  std::shared_ptr<DogBotN::JointC> joint;
  std::cerr << "Joint id " << jointId << "\n";
  if(jointId >= 0) {
    joint = dogbot->GetJointById(jointId);
    if(!joint) {
      logger->error("Failed to find joint id {} ",jointId);
      return 1;
    }
  } else {
    joint = dogbot->GetJointByName(jointName);
    if(!joint) {
      logger->error("Failed to find joint {} ",jointName);
      return 1;
    }
  }

  if(speed >= 0) {
    float omega = 0;
    while(true) {
      omega += 0.01 * speed;
      float goTo = DogBotN::Deg2Rad(angle + (sin(omega) * range/2.0f));
      std::cerr << "Goto:" << goTo << std::endl;
      joint->DemandPosition(goTo,torque);
      usleep(10000);
    }
  } else {
    enum DogBotN::JointMoveStatusT moveStatus;
    while(true)
    {
      if((moveStatus = joint->MoveWait(DogBotN::Deg2Rad(angle-range/2),torque)) != DogBotN::JMS_Done) {
        logger->warn("Move failed. {} ",(int) moveStatus);
      }
      if((moveStatus =joint->MoveWait(DogBotN::Deg2Rad(angle+range/2),torque))!= DogBotN::JMS_Done) {
        logger->warn("Move failed. {} ",(int) moveStatus);
      }
    }
  }

  return 0;
}
