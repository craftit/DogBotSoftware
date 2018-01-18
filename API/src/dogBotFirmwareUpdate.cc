

#include <iostream>
#include <unistd.h>
#include <getopt.h>

#include "dogbot/DogBotAPI.hh"
#include "dogbot/FirmwareUpdate.hh"
#include "cxxopts.hpp"

// This provides a network interface for controlling the servos via ZMQ.

int main(int argc,char **argv)
{
  std::string devFilename = "local";
  std::string configFile;
  std::string firmwareFile;
  int targetDeviceId = 0;
  auto logger = spdlog::stdout_logger_mt("console");

  try
  {
    cxxopts::Options options(argv[0], "DogBot hardware manager");
    options
      .positional_help("[optional args]")
      .show_positional_help();

    options.add_options()
      ("c,config", "Configuration file", cxxopts::value<std::string>(configFile))
      ("d,device", "Device to use from communication ", cxxopts::value<std::string>(devFilename))
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


  logger->info("Starting firmware update");
  logger->info("Using config file: '{}'",configFile);
  logger->info("Using communication type: '{}'",devFilename);

  DogBotN::DogBotAPIC dogbot(
      devFilename,
      configFile,
      logger
      );

  DogBotN::FirmwareUpdateC updater(dogbot.Connection());

  if(!updater.DoUpdate(targetDeviceId,firmwareFile)) {
    logger->error("Firmware update failed");
    return 1;
  }

  logger->info("Firmware update completed ok.");
  return 0;
}
