
#include <iostream>
#include <unistd.h>
#include <getopt.h>

#include "dogbot/ComsSerial.hh"
#include "dogbot/DogBotAPI.hh"
#include "dogbot/ComsZMQServer.hh"
#include "cxxopts.hpp"


int main(int argc,char **argv)
{
  // std::string devFilename = "/dev/tty.usbmodem401";
  // std::string devFilename = "/dev/ttyACM1";
  std::string devFilename = "usb";
  std::string configFile = "";

  bool managerMode = true;
  auto logger = spdlog::stdout_logger_mt("console");

  try
  {
    cxxopts::Options options(argv[0], "Dogbot hardware manager");
    options
      .positional_help("[optional args]")
      .show_positional_help();

    options.add_options()
      ("m,manager", "Manager mode, allowing allocation of device ids", cxxopts::value<bool>(managerMode))
      ("c,config", "Configuration file", cxxopts::value<std::string>(configFile))
      ("d,device", "Device to use from communication ", cxxopts::value<std::string>(devFilename))
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


  logger->info("Starting dogBotServer");
  logger->info("Manager mode: {}",managerMode);
  logger->info("Using config file: '{}'",configFile);
  logger->info("Using device: '{}'",devFilename);

  bool devMaster = true;


  DogBotN::DogBotAPIC dogbot(devFilename,logger,managerMode ? DogBotN::DogBotAPIC::DMM_DeviceManager : DogBotN::DogBotAPIC::DMM_ClientOnly);

  if(!dogbot.Init(configFile)) {
    return 1;
  }

  DogBotN::ComsZMQServerC server(dogbot.Connection(),logger);

  logger->info("Setup and ready. ");

  server.Run("*.*.*.*");

  return 0;
}
