
#include <iostream>
#include <unistd.h>
#include "dogbot/DogBotAPI.hh"
#include "dogbot/ComsUSB.hh"
#include "cxxopts.hpp"

int main(int nargs,char **argv)
{
  std::string devFilename = "usb";
  std::string configFile;
  std::string servoName;
  bool listJoints = false;
  bool listServos = false;

  try
  {
    cxxopts::Options options(argv[0], "Dogbot hardware manager");
    options
      .positional_help("[optional args]")
      .show_positional_help();

    options.add_options()
      ("c,config", "Configuration file", cxxopts::value<std::string>(configFile))
      ("d,device", "Device to use from communication,  'usb' for direct connection or 'local' for a connection to a dobgot server on the local machine.", cxxopts::value<std::string>(devFilename))
      ("l,list", "List available joints ", cxxopts::value<bool>(listJoints))
      ("a,list-servos", "List available servos ", cxxopts::value<bool>(listServos))
      ("s,servo", "Servo to use ", cxxopts::value<std::string>(servoName))
      ("h,help", "Print help")
    ;

    auto result = options.parse(nargs, argv);

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

  DogBotN::DogBotAPIC dogbot(devFilename);

  if(listJoints) {
    std::cout << "Joints: " << std::endl;
    std::vector<std::shared_ptr<DogBotN::JointC> > joints = dogbot.ListJoints();
    for(auto &a : joints) {
      std::cout << a->Name() << std::endl;
    }
  }

  if(listServos) {
    std::cout << "Joints: " << std::endl;
    std::vector<std::shared_ptr<DogBotN::ServoC> > servos = dogbot.ListServos();
    for(auto &a : servos) {
      std::cout << a->Name() << std::endl;
    }
  }

  DogBotN::CallbackSetC cs;

  if(!servoName.empty()) {
    std::shared_ptr<DogBotN::JointC> joint = dogbot.GetJointByName(servoName);
    if(!joint) {
      std::cout << "Joint " << servoName << " not found. " << std::endl;
      return 1;
    }

    cs += joint->AddPositionUpdateCallback([](DogBotN::TimePointT theTime,double position,double velocity,double torque)
                                     {
                                       std::cout << "Joint at " << position << std::endl;
                                     });
  }

  while(1) {
    sleep(1);
  }
  return 0;
}
