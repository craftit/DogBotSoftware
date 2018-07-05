

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <getopt.h>
#include <chrono>

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

  auto logger = spdlog::stdout_logger_mt("console");
  bool cycle = false;
  bool dumpLimits = false;
  int jointId = -1;
  float torque = 1.0;
  float range = 45.0;
  float hight = 0.4;
  float yoffset = 0;
  float xoffset = 0;
  float angle = 0;
  int delay = 5000;
  float speed  = -1.0;
  bool verbose = false;
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
      ("z,height","Motion range", cxxopts::value<float>(hight))
      ("y,yoffset","Forward movement", cxxopts::value<float>(yoffset))
      ("x,xoffset","Sideways movement", cxxopts::value<float>(xoffset))
      ("a,angle","Centre angle", cxxopts::value<float>(angle))
      ("s,speed","Speed for smooth movement", cxxopts::value<float>(speed))
      ("g,cycle","Cycle up and down",cxxopts::value<bool>(cycle))
      ("o,delay","Delay in cycle, default is 5000",cxxopts::value<int>(delay))
      ("n,limits","Dump motion limits",cxxopts::value<bool>(dumpLimits))
      ("v,verbose","Verbose",cxxopts::value<bool>(verbose))
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

  float minLegExtention = dogbot->DogBotKinematics().MinLegExtension();
  float maxLegExtention = dogbot->DogBotKinematics().MaxLegExtension();
  if(hight > maxLegExtention)
    hight = maxLegExtention;
  if(hight < minLegExtention)
    hight = minLegExtention;
  logger->info("Limit Min {}   Max {}  Range:{}",minLegExtention,maxLegExtention,maxLegExtention - minLegExtention);
  if(dumpLimits) { // Just dump limits and exit ?
    return 0;
  }

  // Wait for poses to update and things to settle.
  sleep(1);


  // Lift the speed limit a bit
  dogbot->Connection()->SetParam(0,CPI_VelocityLimit,(float) 500.0);

  std::shared_ptr<DogBotN::LegControllerC> leg = std::make_shared<DogBotN::LegControllerC>(dogbot,"back_right",false);

  if(!cycle) {

    if(!leg->Goto(Eigen::Vector3f(xoffset,yoffset,-hight),torque)) {
      logger->warn("Limit reached.");
    }
    sleep(1);
    for(int i = 0;i < 20 || verbose;i++) {
      //! Compute an estimate of the force on a foot and where it is.
      DogBotN::TimePointT atTime = DogBotN::TimePointT::clock::now();
      Eigen::Vector3f pos;
      Eigen::Vector3f force;
      leg->ComputeFootForce(atTime,pos,force);
      pos -= leg->Kinematics().LegOrigin();
      logger->info("At {} {} {}  force {} {} {} ",pos[0],pos[1],pos[2],force[0],force[1],force[2]);
      usleep(10000);
    }
  } else {
    float midRange = (maxLegExtention+minLegExtention)/2.0;
    while(1) {

      for(int i = 0;i < 360;i++) {
        // 0.35 to 0.7

        float z = cos(DogBotN::Deg2Rad(i)) * hight + midRange;

        leg->Goto(Eigen::Vector3f(0,0,-z),torque);

        usleep(delay);
      }
    }
  }

  return 0;
}
