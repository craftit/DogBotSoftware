
#include "coms.hh"
#include <iostream>
#include <unistd.h>

int main(int nargs,char **argv)
{
  std::string devFilename = "/dev/tty.usbmodem401";

  DogBotN::SerialComsC coms;
  std::ios_base::sync_with_stdio(true);
  std::cerr << "dtalk. " << std::endl;

  if(!coms.Open(devFilename.c_str())) {
    std::cerr<< "Failed to open connection. \n";
    return 1;
  }

  std::cerr << "Setup and ready. " << std::endl;
  while(1) {
    sleep(1);
  }
  return 0;
}
