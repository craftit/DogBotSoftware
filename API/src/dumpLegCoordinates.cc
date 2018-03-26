

#include <iostream>
#include <unistd.h>
#include <getopt.h>

#include "dogbot/DogBotAPI.hh"
#include "cxxopts.hpp"

// This provides a network interface for controlling the servos via ZMQ.

int main(int argc,char **argv)
{
  std::string devFilename = "local";
  std::string configFile = DogBotN::DogBotAPIC::DefaultConfigFile();
  std::string legName = "front_right";
  std::string firmwareFile;


  int targetDeviceId = 0;
  int finalDeviceId = -1;

  bool dryRun = false;
  bool stayInBootloader = false;
  auto logger = spdlog::stdout_logger_mt("console");
  bool updateAll = false;
  try
  {
    cxxopts::Options options(argv[0], "DogBot hardware manager");
    options
      .positional_help("[optional args]")
      .show_positional_help();

    options.add_options()
      ("c,config", "Configuration file", cxxopts::value<std::string>(configFile))
      ("d,device", "Device to use from communication. Typically 'local' for local server or 'usb' for direct connection ", cxxopts::value<std::string>(devFilename))
      ("f,firmware", "Firmware file ", cxxopts::value<std::string>(firmwareFile))
      ("t,target","Target device id", cxxopts::value<int>(targetDeviceId))
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

  std::shared_ptr<DogBotN::LegKinematicsC> leg = dogbot.LegKinematicsByName(legName);
  if(!leg) {
    logger->error("Failed to find leg '{}' ",legName);
  }

  std::vector<std::string> legJointNames;
  legJointNames.push_back(legName + "_roll");
  legJointNames.push_back(legName + "_pitch");
  legJointNames.push_back(legName + "_knee");

  std::shared_ptr<DogBotN::JointC> joints[3];

  for(int i = 0;i < 3;i++) {
    joints[i] = dogbot.GetJointByName(legJointNames[i]);
    if(!joints[i]) {
      logger->error("Failed to find {} joint. ",legJointNames[i]);
      return 1;
    }
  }

  while(true)
  {
    float angles[3];

    for(int i = 0;i < 3;i++) {
      DogBotN::JointC::TimePointT tick;
      double position,velocity,torque;
      joints[i]->GetState(tick,position,velocity,torque);
      angles[i] = position;
    }

    float at[3];
    leg->Forward(angles,at);

    logger->info("{} at {} {} {} ",legName,at[0],at[1],at[2]);

    sleep(1);
  }

  logger->info("Firmware update completed ok.");
  return 0;
}
