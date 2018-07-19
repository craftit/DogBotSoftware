
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <sstream>
#include <thread>
#include <vector>
#include <functional>
#include <assert.h>
#include <mutex>

#include "dogbot/DogBotControllerROS.hh"
#include "dogbot/SplineGaitController.hh"
#include "dogbot/DogBotAPI.hh"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dogbot_simplegait");
  ros::NodeHandle n;
  ros::NodeHandle nPrivate("~");

  std::string robotns = "dogbot";
  nPrivate.getParam("robot",robotns);

  float loopHz = 100;
  float omega = 0.5;
  nPrivate.getParam("loop_hz",loopHz);
  nPrivate.getParam("omega",omega);


  ros::AsyncSpinner spinner(2);

  sensor_msgs::JointState::ConstPtr lastJointState;

  std::shared_ptr<DogBotN::DogBotKinematicsC> dogBotKinematics = std::make_shared<DogBotN::DogBotKinematicsC>();

  if(!dogBotKinematics->LoadConfig(DogBotN::DogBotAPIC::DefaultConfigFile(""))) {
    std::cerr << "Failed to load robot config. " << std::endl;
  }

  DogBotN::DogBotControllerROSC legs(robotns,dogBotKinematics);

  float updatePeriod = 1.0f/loopHz;
  legs.SetupTrajectory(updatePeriod,15);

  DogBotN::SplineGaitControllerC gaitGenerator;
  gaitGenerator.SetOmega(omega);

  std::cout << "Starting controller. Updated.  LoopHz:" << loopHz << " Omega:" << omega << std::endl;

  ros::Timer timer1 = n.createTimer(ros::Duration(updatePeriod), [&legs,&gaitGenerator,updatePeriod](const ros::TimerEvent &te){
    //! Do a single timestep
    std::cout << "Update. " << std::endl;
    DogBotN::SimpleQuadrupedPoseC pose;
    gaitGenerator.Step(updatePeriod,pose);
    legs.NextTrajectory(pose);
  });


  // Wait until shutdown signal received
  spinner.start();
  ros::waitForShutdown();

  std::cout << "Done " << std::endl;

  return 0;
}
