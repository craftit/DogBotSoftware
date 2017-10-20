

#include "dogbot/LegKinematics.hh"
#include <iostream>
#include <math.h>

static float deg2rad(float deg)
{
  return ((float) deg *2.0 * M_PI)/ 360.0;
}

static float rad2deg(float deg)
{
  return ((float) deg * 360.0) / (2.0 * M_PI);
}

int main() {
  DogBotN::LegKinematicsC legKinamtics(0.36,0.31);

  float angles[3];
  float target[3] = {0.15,0.1,0.3};

  if(!legKinamtics.Inverse(target,angles)) {
    std::cerr << "Failed." << std::endl;
    return 1;
  }
  std::cout << angles[0] << " " << angles[1] << " " << angles[2] << " " << std::endl;

  float pos[3];
  if(!legKinamtics.Forward(angles,pos)) {
    std::cerr << "Forward kinematics failed." << std::endl;
    return 1;
  }

  std::cout <<" Target: "<< target[0] << " " << target[1] << " " << target[2] << " " << std::endl;
  std::cout <<" At: "<< pos[0] << " " << pos[1] << " " << pos[2] << " " << std::endl;

  float angles2[3] = {0,0,0};
  if(!legKinamtics.Forward(angles2,pos)) {
    std::cerr << "Forward kinematics failed." << std::endl;
    return 1;
  }

  std::cout <<" 0,0,0 at: "<< pos[0] << " " << pos[1] << " " << pos[2] << " " << std::endl;

  for(int i = 0;i < 360;i+=10) {
    float angle = deg2rad((float) i);
    float out = legKinamtics.Linkage4BarForward(angle);
    float back = 0;
    bool ok = legKinamtics.Linkage4BarBack(out,back);
    std::cout << i << " Fwd:" << rad2deg(out) << " Inv:" << rad2deg(back) << "  (" << ok << ")" << std::endl;
  }

  return 0;
}
