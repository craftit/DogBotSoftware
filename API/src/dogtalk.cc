
#include <iostream>
#include <unistd.h>
#include "dogbot/DogBotAPI.hh"

int main(int nargs,char **argv)
{
  auto logger = spdlog::stdout_logger_mt("console");

  logger->info("Starting dtalk");


  //std::string devFilename = "/dev/tty.usbmodem401";
  std::string configFile = "dogbot.conf";
  if(nargs > 1)
    configFile = argv[1];

  DogBotN::DogBotAPIC dogbot;

  dogbot.SetLogger(logger);

  if(!dogbot.Init(configFile)) {
    return 1;
  }

  logger->info("Setup and ready. ");
  while(1) {
    sleep(1);
  }
  return 0;
}
