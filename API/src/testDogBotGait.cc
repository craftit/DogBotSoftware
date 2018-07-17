

#include "dogbot/SplineGaitController.hh"

int testSplineGait() {
  DogBotN::SplineGaitControllerC gait;
  gait.PlotGait();
  return 0;
}


int main(int nargs,char **argv)
{
  testSplineGait();
  return 0;
}
