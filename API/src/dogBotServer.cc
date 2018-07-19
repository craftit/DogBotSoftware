
#include <iostream>
#include <unistd.h>
#include <getopt.h>

#include "dogbot/ComsSerial.hh"
#include "dogbot/DogBotAPI.hh"
#include "dogbot/ComsZMQServer.hh"
#include "dogbot/ComsRecorder.hh"
#include "dogbot/ComsRoute.hh"
#include "dogbot/PlatformManager.hh"
#include "cxxopts.hpp"
#include <sched.h>

#define DODEBUG 0
#if DODEBUG
#define ONDEBUG(x) x
#else
#define ONDEBUG(x)
#endif

// This provides a network interface for controlling the servos via ZMQ.

int main(int argc,char **argv)
{
  std::string devFilename = "usb";
  std::string devIMUFilename = ""; //"/dev/ttyUSB1";
  std::string configFile = DogBotN::DogBotAPIC::DefaultConfigFile();
  std::string logFile;
  std::string zmqAddress = "tcp://*";
  bool managerMode = true;
  auto logger = spdlog::stdout_logger_mt("console");

  struct sched_param params;
  params.sched_priority = 50;

  // This should
#if defined(__APPLE__)
#elif defined(__linux__)
  int ret;
  if((ret = sched_setscheduler(0,SCHED_RR,&params)) < 0) {
    logger->warn("Failed to set priority. Error:{} ",strerror(errno));
  }
#endif

  try
  {
    cxxopts::Options options(argv[0], "DogBot hardware manager");
    options
      .positional_help("[optional args]")
      .show_positional_help();

    options.add_options()
      ("m,manager", "Manager mode, allowing allocation of device ids", cxxopts::value<bool>(managerMode))
      ("c,config", "Configuration file", cxxopts::value<std::string>(configFile))
      ("d,device", "Device to use for communication ", cxxopts::value<std::string>(devFilename))
      ("i,imu",    "Device to use for IMU communication ", cxxopts::value<std::string>(devIMUFilename))
      ("l,log"   , "File to use for communication log ", cxxopts::value<std::string>(logFile))
      ("n,net"   , "Address to offer service on ", cxxopts::value<std::string>(zmqAddress))
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


  ONDEBUG(logger->info("Starting dogBotServer"));
  logger->info("Manager mode: {}",managerMode);
  logger->info("Using config file: '{}'",configFile);
  logger->info("Using communication type: '{}'",devFilename);
  logger->info("Communication log: '{}'",logFile);

  std::shared_ptr<DogBotN::ComsRouteC> coms = std::make_shared<DogBotN::ComsRouteC>();


  if(!coms->Open(devFilename)) {
    logger->error("Failed to open {} ",devFilename);
    return 1;
  }
  if(!devIMUFilename.empty()) {
    if(!coms->Open(devIMUFilename)) {
      logger->error("Failed to open {} ",devIMUFilename);
      return 1;
    }
  }

  std::shared_ptr<DogBotN::ComsC> pureComs(coms);

  std::shared_ptr<DogBotN::DogBotAPIC> dogbot = std::make_shared<DogBotN::DogBotAPIC>(
      pureComs,
      DogBotN::DogBotAPIC::DefaultConfigFile(),
      logger,
      false,
      managerMode ? DogBotN::DogBotAPIC::DMM_DeviceManager : DogBotN::DogBotAPIC::DMM_ClientOnly
          );

  if(1) {
    std::shared_ptr<DogBotN::PlatformManagerC> platformManager = std::make_shared<DogBotN::PlatformManagerC>(dogbot);
    coms->AddComs(platformManager);
    platformManager->Init();
  }

  DogBotN::ComsZMQServerC server(pureComs,logger);

  std::shared_ptr<DogBotN::ComsRecorderC> logRecorder;
  if(!logFile.empty()) {
    logRecorder = std::make_shared<DogBotN::ComsRecorderC>(pureComs,logger,logFile);
    logRecorder->Start();
  }

  ONDEBUG(logger->info("Setup and ready. "));

  server.Run(zmqAddress);

  return 0;
}
