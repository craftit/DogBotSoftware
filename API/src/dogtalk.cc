
#include <iostream>
#include <unistd.h>
#include "dogbot/DogBotAPI.hh"
#include "dogbot/ComsUSB.hh"

int main(int nargs,char **argv)
{
  auto logger = spdlog::stdout_logger_mt("console");

  logger->info("Starting dtalk");

  std::shared_ptr<DogBotN::ComsC> comsPtr = std::make_shared<DogBotN::ComsUSBC>(logger);
#if 0
  //std::string devFilename = "/dev/tty.usbmodem401";
  std::string configFile = "dogbot.conf";
  if(nargs > 1)
    configFile = argv[1];

  DogBotN::DogBotAPIC dogbot;

  dogbot.SetLogger(logger);

  if(!dogbot.Init(configFile)) {
    return 1;
  }
#endif

  logger->info("Setup and ready. ");
  while(1) {
    sleep(1);
  }
  return 0;
}
