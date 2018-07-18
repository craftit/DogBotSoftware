
// This program if for checking the kinematics class is producing sensible results.
// It should be turned into a full unit test.

#include "dogbot/LegKinematics.hh"
#include "dogbot/Util.hh"
#include <iostream>
#include <math.h>

DogBotN::LegKinematicsC g_legKinematics;

static float deg2rad(float deg)
{
  return ((float) deg *2.0 * M_PI)/ 360.0;
}

static float rad2deg(float deg)
{
  return ((float) deg * 360.0) / (2.0 * M_PI);
}

float Diff(const DogBotN::LegKinematicsC &legKinematics,float theta) {
  float delta = 1e-4;
  float psi = legKinematics.Linkage4BarForward(theta);
  float psiD = legKinematics.Linkage4BarForward(theta+delta);
  return delta/(psiD-psi);
  //return (psiD-psi)/delta;
}

// Generate a table of servo and joint angles.

int CheckLinkageAngles() {

  for(int i = 0;i < 360;i+=10) {
    float theta = deg2rad((float) i);
    float psi = g_legKinematics.Linkage4BarForward(theta);
    float back = 0;
    float back2 = 0;
    bool ok = g_legKinematics.Linkage4BarBack(psi,back,false);
    g_legKinematics.Linkage4BarBack(psi,back2,true);
    float ratio1 = g_legKinematics.LinkageSpeedRatio(back,psi);
    float ratio2 = g_legKinematics.LinkageSpeedRatio(back2,psi);
    float diff1 = Diff(g_legKinematics,back);
    float diff2 = Diff(g_legKinematics,back2);
    std::cout << i << " Fwd:" << rad2deg(psi) << " Inv:" << rad2deg(back)  << " (" << ratio1 << " " << diff1 << ") "
                                                         << rad2deg(back2) << " (" << ratio2 << " " << diff2 << ") " << "  [" << ok << "]" << std::endl;
  }

  return 0;
}


int TestFixedAngles()
{
  {
    std::cout << "Zero:" << std::endl;
    Eigen::Vector3f pos;
    Eigen::Vector3f angles = {0,M_PI/2.0,M_PI/2};
    if(!g_legKinematics.ForwardVirtual(angles,pos)) {
      std::cerr << "Forward kinematics failed." << std::endl;
      return 1;
    }
    std::cout << " Angles:" << DogBotN::Rad2Deg(angles[0]) << " " << DogBotN::Rad2Deg(angles[1]) << " " << DogBotN::Rad2Deg(angles[2]) << " " << std::endl;
    std::cout << "     At: "<< pos[0] << " " << pos[1] << " " << pos[2] << " " << std::endl;
  }
  {
    Eigen::Vector3f pos;
    Eigen::Vector3f angles2 = {0,0,0};
    if(!g_legKinematics.ForwardVirtual(angles2,pos)) {
      std::cerr << "Forward kinematics failed." << std::endl;
      return 1;
    }
    std::cout <<" 0,0,0 at: "<< pos[0] << " " << pos[1] << " " << pos[2] << " " << std::endl;
  }
  return 0;
}

// Check inverse and forward kinematics agree for a virtual joint

int CheckTargetVirtual(Eigen::Vector3f target) {
  DogBotN::LegKinematicsC legKinematics;
  Eigen::Vector3f angles;
  Eigen::Vector3f pos;

  if(!g_legKinematics.InverseVirtual(target,angles)) {
    std::cerr << "Failed." << std::endl;
    return __LINE__;
  }
  std::cout << " Virtual Angles:" << DogBotN::Rad2Deg(angles[0]) << " " << DogBotN::Rad2Deg(angles[1]) << " " << DogBotN::Rad2Deg(angles[2]) << " " << std::endl;

  if(!g_legKinematics.ForwardVirtual(angles,pos)) {
    std::cerr << "Forward kinematics failed." << std::endl;
    return __LINE__;
  }

  float dist = (target - pos).norm();
  std::cout <<" Target: "<< target[0] << " " << target[1] << " " << target[2] << " " << std::endl;
  std::cout <<" At: "<< pos[0] << " " << pos[1] << " " << pos[2] << "   Distance:" << dist << std::endl;
  if(dist > 1e-6) {
    return __LINE__;
  }
  return 0;
}

// Check inverse and forward kinematics agree for a direct joint

int CheckTargetDirect(Eigen::Vector3f target,bool verbose = true) {
  DogBotN::LegKinematicsC legKinematics;
  Eigen::Vector3f angles;
  Eigen::Vector3f pos;

  if(!g_legKinematics.InverseDirect(target,angles)) {
    if(verbose)
      std::cerr << "Failed to find solution for target :" << target << " " << std::endl;
    return __LINE__;
  }
  if(verbose)
    std::cout << " Direct Angles:" << DogBotN::Rad2Deg(angles[0]) << " " << DogBotN::Rad2Deg(angles[1]) << " " << DogBotN::Rad2Deg(angles[2]) << " " << std::endl;

  if(!g_legKinematics.ForwardDirect(angles,pos)) {
    std::cerr << "Forward kinematics failed." << std::endl;
    return __LINE__;
  }

  float dist = (target - pos).norm();
  if(verbose) {
    std::cout <<" Target: "<< target[0] << " " << target[1] << " " << target[2] << " " << std::endl;
    std::cout <<" At: "<< pos[0] << " " << pos[1] << " " << pos[2] << "   Distance:" << dist << std::endl;
  }
  if(dist > 1e-6) {
    return __LINE__;
  }
  return 0;
}

//! Check various positions in the foot space a reachable.

int TestReachableTargets()
{
  int ln =0;

  {
    Eigen::Vector3f target = {0.15,0.1,0.3};
    if((ln = CheckTargetDirect(target)) != 0) {
      std::cerr << "Check target failed at " << ln << std::endl;
      return __LINE__;
    }
  }

  float maxReach;
  {
    maxReach = g_legKinematics.MaxExtension();
    Eigen::Vector3f target = {0,0,maxReach};
    if((ln = CheckTargetDirect(target)) != 0) {
      std::cerr << "Check target failed at " << ln << std::endl;
      return __LINE__;
    }
  }

  float minReach;
  {
    minReach = g_legKinematics.MinExtension();
    Eigen::Vector3f target = {0,0,minReach};
    if((ln = CheckTargetDirect(target)) != 0) {
      std::cerr << "Check target failed at " << ln << std::endl;
      return __LINE__;
    }
  }

  {
    std::cout << "Stride at max " << (maxReach) << " is " << g_legKinematics.StrideLength(maxReach) << std::endl;
    {
      float i = maxReach;
      float stride = g_legKinematics.StrideLength(i);
      std::cout << "Stride at " << i << " is " << stride << std::endl;

      Eigen::Vector3f target = {0,(float) (stride/2),i};
      if((ln = CheckTargetDirect(target,false)) != 0) {
        std::cerr << "Check target failed at " << ln << std::endl;
        //return __LINE__;
      }
    }

    std::cout << "Stride at min " << (minReach) << " is " << g_legKinematics.StrideLength(minReach) << std::endl;
  }

  for(float i = minReach;i <= maxReach;i += 0.05) {
    float stride = g_legKinematics.StrideLength(i);
    std::cout << "Stride at " << i << " is " << stride << std::endl;

    Eigen::Vector3f target = {0,(float) (stride/2),i};
    if((ln = CheckTargetDirect(target,false)) != 0) {
      std::cerr << "Check target failed at " << ln << std::endl;
      //return __LINE__;
    }
  }

  return 0;
}



int main() {
  int ln = 0;
#if 1
  if((ln = CheckLinkageAngles()) != 0) {
    std::cerr << "Test failed at " << ln << " " << std::endl;
    return 1;
  }
  if((ln = TestFixedAngles()) != 0) {
    std::cerr << "Test failed at " << ln << " " << std::endl;
    return 1;
  }
#endif
  if((ln = TestReachableTargets()) != 0) {
    std::cerr << "Test failed at " << ln << " " << std::endl;
    return 1;
  }
  std::cerr << "Test passed." << std::endl;
  return 0;
}
