
// This program if for checking the kinematics class is producing sensible results.
// It should be turned into a full unit test.

#include "dogbot/LegKinematics.hh"
#include "dogbot/Util.hh"
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

float Diff(const DogBotN::LegKinematicsC &legKinamtics,float theta) {
  float delta = 1e-4;
  float psi = legKinamtics.Linkage4BarForward(theta);
  float psiD = legKinamtics.Linkage4BarForward(theta+delta);
  return delta/(psiD-psi);
  //return (psiD-psi)/delta;
}

int main() {
  DogBotN::LegKinematicsC legKinamtics(0.36,0.31);

  float pos[3];
  {
    float angles[3];
    float target[3] = {0.15,0.1,0.3};

    if(!legKinamtics.InverseVirtual(target,angles)) {
      std::cerr << "Failed." << std::endl;
      return 1;
    }
    std::cout << " Angles:" << DogBotN::Rad2Deg(angles[0]) << " " << DogBotN::Rad2Deg(angles[1]) << " " << DogBotN::Rad2Deg(angles[2]) << " " << std::endl;

    if(!legKinamtics.ForwardVirtual(angles,pos)) {
      std::cerr << "Forward kinematics failed." << std::endl;
      return 1;
    }
    std::cout <<" Target: "<< target[0] << " " << target[1] << " " << target[2] << " " << std::endl;
    std::cout <<" At: "<< pos[0] << " " << pos[1] << " " << pos[2] << " " << std::endl;


  }
  {
    std::cout << "Zero:" << std::endl;
    float angles[3] = {0,M_PI/2.0,M_PI/2};
    if(!legKinamtics.ForwardVirtual(angles,pos)) {
      std::cerr << "Forward kinematics failed." << std::endl;
      return 1;
    }
    std::cout << " Angles:" << DogBotN::Rad2Deg(angles[0]) << " " << DogBotN::Rad2Deg(angles[1]) << " " << DogBotN::Rad2Deg(angles[2]) << " " << std::endl;
    std::cout << "     At: "<< pos[0] << " " << pos[1] << " " << pos[2] << " " << std::endl;
  }
  {
    float angles2[3] = {0,0,0};
    if(!legKinamtics.ForwardVirtual(angles2,pos)) {
      std::cerr << "Forward kinematics failed." << std::endl;
      return 1;
    }
    std::cout <<" 0,0,0 at: "<< pos[0] << " " << pos[1] << " " << pos[2] << " " << std::endl;
  }


  for(int i = 0;i < 360;i+=10) {
    float theta = deg2rad((float) i);
    float psi = legKinamtics.Linkage4BarForward(theta);
    float back = 0;
    float back2 = 0;
    bool ok = legKinamtics.Linkage4BarBack(psi,back);
    legKinamtics.Linkage4BarBack(psi,back2,true);
    float ratio1 = legKinamtics.LinkageSpeedRatio(back,psi);
    float ratio2 = legKinamtics.LinkageSpeedRatio(back2,psi);
    float diff1 = Diff(legKinamtics,back);
    float diff2 = Diff(legKinamtics,back2);
    std::cout << i << " Fwd:" << rad2deg(psi) << " Inv:" << rad2deg(back) << " (" << ratio1 << " " << diff1 << ") " << rad2deg(back2) << " (" << ratio2 << " " << diff2 << ") " << "  [" << ok << "]" << std::endl;
  }

  return 0;
}
