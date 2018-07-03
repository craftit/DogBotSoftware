

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <getopt.h>

#include "dogbot/DogBotAPI.hh"
#include "dogbot/Util.hh"
#include "dogbot/LegController.hh"
#include "dogbot/SplineGaitController.hh"
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
  float torque = 3.0;
  float range = 45.0;
  float angle = 0;
  float velocityLimit = 300.0;
  std::string gaitType = "walk";
  bool plotGait = false;
  try
  {
    cxxopts::Options options(argv[0], "DogBot hardware manager");
    options
      .positional_help("[optional args]")
      .show_positional_help();

    options.add_options()
      ("c,config", "Configuration file", cxxopts::value<std::string>(configFile))
      ("d,device", "Device to use from communication. Typically 'local' for local server or 'usb' for direct connection ", cxxopts::value<std::string>(devFilename))
      ("j,joint","Joint name", cxxopts::value<std::string>(jointName))
      ("t,torque","Maximum torque to apply", cxxopts::value<float>(torque))
      ("v,velocity","Maximum velocity to allow",cxxopts::value<float>(velocityLimit))
      ("g,gait","Gait type, 'walk' or 'trot' ",cxxopts::value<std::string>(gaitType))
      ("p,plot","Plot gait. ",cxxopts::value<bool>(plotGait))
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

  DogBotN::SplineGaitControllerC gaitController;

  if(!gaitType.empty()) {
    gaitController.SetStyle(gaitType);
  }

  if(plotGait) {
    gaitController.PlotGait();
    return 1;
  }

  std::shared_ptr<DogBotN::DogBotAPIC> dogbot = std::make_shared<DogBotN::DogBotAPIC>(
      devFilename,
      configFile,
      logger
      );



  // Wait for poses to update and things to settle.
  sleep(1);

  {
    // Lift the speed limit a bit
    dogbot->Connection()->SetParam(0,CPI_VelocityLimit,velocityLimit);

    std::shared_ptr<DogBotN::LegControllerC> legs[4];
    legs[0] = std::make_shared<DogBotN::LegControllerC>(dogbot,"front_left");
    legs[1] = std::make_shared<DogBotN::LegControllerC>(dogbot,"front_right");
    legs[2] = std::make_shared<DogBotN::LegControllerC>(dogbot,"back_left");
    legs[3] = std::make_shared<DogBotN::LegControllerC>(dogbot,"back_right");

    while(1) {


      DogBotN::SimpleQuadrupedPoseC pose;
      gaitController.Step(0.01,pose);

      for(int i = 0;i < 4;i++) {
        legs[i]->Goto(pose.FootPosition(i),torque);
      }

      usleep(10000); // ~100Hz
    }

  }



  return 0;
}
