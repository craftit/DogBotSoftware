
#include <iostream>
#include <unistd.h>

#include "dogbot/ComsSerial.hh"
#include "dogbot/DogBotAPI.hh"
#include "dogbot/ComsZMQServer.hh"

int main(int nargs,char **argv)
{
  auto logger = spdlog::stdout_logger_mt("console");

  logger->info("Starting dogBotServer");

  bool devMaster = true;

//  std::string devFilename = "/dev/tty.usbmodem401";
  std::string devFilename = "/dev/ttyACM1";
  std::string configFile = "";
  if(nargs > 1)
    configFile = argv[1];

  DogBotN::DogBotAPIC dogbot(devFilename,logger,DogBotN::DogBotAPIC::DMM_DeviceManager);

  if(!dogbot.Init(configFile)) {
    return 1;
  }

  DogBotN::ComsZMQServerC server(dogbot.Connection(),logger);

  logger->info("Setup and ready. ");

  server.Run("*.*.*.*");

  return 0;
}
