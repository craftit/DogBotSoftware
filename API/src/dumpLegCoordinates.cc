

#include <iostream>
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
  std::string legName = "front_right";

  Eigen::Vector3f at = { 0,0,0.6 };

  float torqueLimit = 1.0;

  bool dryRun = false;
  auto logger = spdlog::stdout_logger_mt("console");
  try
  {
    cxxopts::Options options(argv[0], "DogBot hardware manager");
    options
      .positional_help("[optional args]")
      .show_positional_help();

    options.add_options()
      ("c,config", "Configuration file", cxxopts::value<std::string>(configFile))
      ("d,device", "Device to use from communication. Typically 'local' for local server or 'usb' for direct connection ", cxxopts::value<std::string>(devFilename))
      ("t,torque","Torque", cxxopts::value<float>(torqueLimit))
      ("l,leg","leg", cxxopts::value<std::string>(legName))
      ("x","x", cxxopts::value<float>(at[0]))
      ("y","y", cxxopts::value<float>(at[1]))
      ("z","z", cxxopts::value<float>(at[2]))
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

  sleep(1);
  std::shared_ptr<DogBotN::LegKinematicsC> leg = dogbot.LegKinematicsByName(legName);
  if(!leg) {
    logger->error("Failed to find leg '{}' ",legName);
  }

  std::vector<std::string> legJointNames;
  legJointNames.push_back(legName + "_roll");
  legJointNames.push_back(legName + "_pitch");
  legJointNames.push_back("virtual_" + legName + "_knee");

  std::shared_ptr<DogBotN::JointC> joints[3];

  for(int i = 0;i < 3;i++) {
    joints[i] = dogbot.GetJointByName(legJointNames[i]);
    if(!joints[i]) {
      logger->error("Failed to find {} joint. ",legJointNames[i]);
      return 1;
    }
  }

  Eigen::Vector3f angles;
  if(leg->InverseVirtual(at,angles)) {
    logger->info("Setting angles to {} {} {}  for  {} {} {} ",
                 DogBotN::Rad2Deg(angles[0]),DogBotN::Rad2Deg(angles[1]),DogBotN::Rad2Deg(angles[2]),
                 at[0],at[1],at[2]);
    joints[0]->DemandPosition(angles[0],torqueLimit);
    joints[1]->DemandPosition(angles[1],torqueLimit);
    joints[2]->DemandPosition(angles[2],torqueLimit);

  } else {
    logger->warn("Failed to find solution.");
  }


  while(true)
  {

    for(int i = 0;i < 3;i++) {
      DogBotN::TimePointT tick;
      double position,velocity,torque;
      joints[i]->GetState(tick,position,velocity,torque);
      angles[i] = position;
    }

    Eigen::Vector3f at;
    leg->ForwardVirtual(angles,at);

    logger->info("{} Angles {} {} {}, Position {} {} {}  ",
                 legName,
                 DogBotN::Rad2Deg(angles[0]),DogBotN::Rad2Deg(angles[1]),DogBotN::Rad2Deg(angles[2]),at[0],at[1],at[2]);

    sleep(1);
  }

  logger->info("Firmware update completed ok.");
  return 0;
}
