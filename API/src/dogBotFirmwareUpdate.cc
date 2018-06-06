

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
      ("t,target","Target device id, should always be set to the bridge device id.", cxxopts::value<int>(targetDeviceId))
      ("a,all","Update all connected devices ", cxxopts::value<bool>(updateAll))
      ("n,dryrun","Dry run", cxxopts::value<bool>(dryRun))
      ("e,noexit","Stay in boot-loader after update is complete.", cxxopts::value<bool>(stayInBootloader))
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
  if(dryRun)
    updater.SetDryRun();
  updater.SetExitBootloaderOnComplete(!stayInBootloader);

  if(updateAll) {
    sleep(5); // Wait for device id's to be updated.
    std::vector<std::shared_ptr<DogBotN::ServoC> > servos = dogbot.ListServos();
    for(auto &a : servos) {
      if(!a)
        continue;
      bool done = false;
      if(targetDeviceId == a->Id())
        continue; // Do the targeted device last
      logger->info("Updating target {} ",a->Id());
      for(int i = 0;i < 2;i++) {
        if(updater.DoUpdate(a->Id(),firmwareFile)) {
          done = true;
          break;
        }
      }
      if(!done) {
        logger->error("Firmware update failed for target {} ",targetDeviceId);
        break;
      }
    }
  }

  {
    logger->info("Updating target {} ",targetDeviceId);
    bool done = false;
    for(int i = 0;i < 2;i++) {
      if(updater.DoUpdate(targetDeviceId,firmwareFile)) {
        done = true;
        break;
      }
    }
    if(!done) {
      logger->error("Firmware update failed for target {} ",targetDeviceId);
      return 1;
    }
  }

  logger->info("Firmware update completed ok.");
  return 0;
}
