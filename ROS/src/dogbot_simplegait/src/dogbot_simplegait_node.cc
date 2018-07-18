
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
  ros::init(argc, argv, "dogbot_locomotion");
  ros::NodeHandle n;
  ros::NodeHandle nPrivate("~");

  std::string robotns = "dogbot";
  nPrivate.getParam("robot",robotns);

  sensor_msgs::JointState::ConstPtr lastJointState;

  std::shared_ptr<DogBotN::DogBotKinematicsC> dogBotKinematics = std::make_shared<DogBotN::DogBotKinematicsC>();

  if(!dogBotKinematics->LoadConfig(DogBotN::DogBotAPIC::DefaultConfigFile(""))) {
    std::cerr << "Failed to load robot config. " << std::endl;
  }

  DogBotN::DogBotControllerROSC legs(robotns,dogBotKinematics);

  float defaultUpdatePeriod = 0.02;
  legs.SetupTrajectory(defaultUpdatePeriod,15);

  DogBotN::SplineGaitControllerC gaitGenerator;

  std::string jointStateTopic = robotns + "/joint_states";

  std::cout << "Starting controller. Updated" << std::endl;


  ros::Subscriber subJoints = n.subscribe<sensor_msgs::JointState>(jointStateTopic, 3,
    [&lastJointState,&legs,&gaitGenerator,defaultUpdatePeriod](const sensor_msgs::JointState::ConstPtr& msg) mutable
    {
      float diffTime = defaultUpdatePeriod;
      if(lastJointState) {
        diffTime = (msg->header.stamp - lastJointState->header.stamp).toSec();
      }
      lastJointState = msg;
      std::cout << "At " << diffTime << std::endl;

      //! Do a single timestep
      DogBotN::SimpleQuadrupedPoseC pose;
      gaitGenerator.Step(diffTime,pose);
      legs.NextTrajectory(pose);
    }
  );

  ros::spin();

  std::cout << "Done " << std::endl;

  return 0;
}
