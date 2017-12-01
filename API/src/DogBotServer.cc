
#include <iostream>
#include <unistd.h>

#include "../include/dogbot/ComsSerial.hh"
#include "dogbot/DogBotAPI.hh"
#include "dogbot/ComsZMQServer.hh"

int main(int nargs,char **argv)
{
  auto logger = spdlog::stdout_logger_mt("console");

  logger->info("Starting dtalk");


  std::string devFilename = "/dev/tty.usbmodem401";
  std::string configFile = "dogbot.conf";
  if(nargs > 1)
    configFile = argv[1];

  std::shared_ptr<DogBotN::ComsC> coms = std::make_shared<DogBotN::ComsSerialC>(devFilename.c_str());

  DogBotN::DogBotAPIC dogbot(coms,logger);

  if(!dogbot.Init(configFile)) {
    return 1;
  }

  DogBotN::ComsZMQServerC server(coms,logger);

  logger->info("Setup and ready. ");

  server.Run("*.*.*.*");

  return 0;
}
