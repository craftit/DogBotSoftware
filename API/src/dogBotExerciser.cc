

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <getopt.h>

#include "dogbot/DogBotAPI.hh"
#include "dogbot/Util.hh"
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
  float torque = 1.0;
  try
  {
    cxxopts::Options options(argv[0], "DogBot hardware manager");
    options
      .positional_help("[optional args]")
      .show_positional_help();

    options.add_options()
      ("c,config", "Configuration file", cxxopts::value<std::string>(configFile))
      ("d,device", "Device to use from communication. Typically 'local' for local server or 'usb' for direct connection ", cxxopts::value<std::string>(devFilename))
      ("t,torque","Torque to use", cxxopts::value<float>(torque))
      ("j,joint","Joint name", cxxopts::value<std::string>(jointName))
      ("p,pose","Joint name", cxxopts::value<bool>(dumpPose))
      ("l,load","Load pose", cxxopts::value<std::string>(loadPoseFile))
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

  DogBotN::DogBotAPIC dogbot(
      devFilename,
      configFile,
      logger
      );

  // Wait for poses to update and things to settle.
  sleep(1);

  if(dumpPose) {
    std::vector<std::shared_ptr<DogBotN::ServoC> > list = dogbot.ListServos();
    std::cout << "{" << std::endl;
    for(auto &a : list) {
      if(!a)
        continue;
      std::cout << " \"" << a->Name() << "\":" << DogBotN::Rad2Deg(a->Position()) << "," << std::endl;
    }
    std::cout << "}" << std::endl;
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
      std::shared_ptr<DogBotN::JointC> jnt = dogbot.GetJointByName(iter.key().asString());
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

  std::shared_ptr<DogBotN::JointC> joint = dogbot.GetJointByName(jointName);
  if(!joint) {
    logger->error("Failed to find joint {} ",jointName);
    return 1;
  }

  enum DogBotN::JointMoveStatusT moveStatus;
  while(true)
  {
    if((moveStatus = joint->MoveWait(DogBotN::Deg2Rad(-22.5),torque)) != DogBotN::JMS_Done) {
      logger->warn("Move failed. {} ",(int) moveStatus);
    }
    if((moveStatus =joint->MoveWait(DogBotN::Deg2Rad(22.5),torque))!= DogBotN::JMS_Done) {
      logger->warn("Move failed. {} ",(int) moveStatus);
    }
  }

  return 0;
}
