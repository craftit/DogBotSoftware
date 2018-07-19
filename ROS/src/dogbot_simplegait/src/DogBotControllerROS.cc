



#include "dogbot/DogBotControllerROS.hh"
#include "dogbot/Util.hh"
#include "dogbot/LineABC2d.hh"
#include "dogbot/DogBotKinematics.hh"

namespace DogBotN {

  DogBotControllerROSC::DogBotControllerROSC()
  {}


  DogBotControllerROSC::DogBotControllerROSC(
      const std::string &name,
      const std::shared_ptr<DogBotKinematicsC> &dogBotKinematics
      )
   : m_dogBotKinematics(dogBotKinematics),
     m_robotNamespace(name)
  {
    Init();
  }

  //! Destructor
  DogBotControllerROSC::~DogBotControllerROSC()
  {}

  void DogBotControllerROSC::Init()
  {
    //const std::vector<std::string> &legNames = DogBotKinematicsC::LegNames();

    m_jointControllers = std::vector<ros::Publisher>(12);

    for(int i = 0;i < 4;i++) {
      std::string legName = DogBotN::DogBotKinematicsC::LegNames()[i];

      int off = i * 3;
      m_jointControllers[off + 0] = m_node.advertise<std_msgs::Float64>(m_robotNamespace + "/" +legName + "_roll_position_controller/command", 3);
      m_jointControllers[off + 1] = m_node.advertise<std_msgs::Float64>(m_robotNamespace + "/" +legName + "_pitch_position_controller/command", 3);
      m_jointControllers[off + 2] = m_node.advertise<std_msgs::Float64>(m_robotNamespace + "/" +legName + "_knee_position_controller/command", 3);
    }

    std::string jointStateTopic = m_robotNamespace + "/joint_states";

    m_polarity = std::vector<float>(12);
    m_polarity[0] = 1; // Hip
    m_polarity[1] = 1; // Leg
    m_polarity[2] = 1; // Knee

    m_polarity[3] = 1;
    m_polarity[4] = 1;
    m_polarity[5] = 1;

    m_polarity[6] = 1;
    m_polarity[7] = 1;
    m_polarity[8] = 1;

    m_polarity[9] = 1;
    m_polarity[10] = 1;
    m_polarity[11] = 1;
  }


  //! Setup trajectory
  bool DogBotControllerROSC::SetupTrajectory(float updatePeriod,float torqueLimit)
  {
    return true;
  }

  //! Send next trajectory position, this should be called at 'updatePeriod' intervals as setup
  //! with SetupTrajectory.
  bool DogBotControllerROSC::NextTrajectory(const PoseAnglesC &pose)
  {
    for(int i = 0;i < 12;i++) {
      std_msgs::Float64 msg;
      msg.data = pose.JointPosition(i) * m_polarity[i];
      //std::cout << " Joint " << i << " = " << msg.data << std::endl;
      m_jointControllers[i].publish(msg);
    }
    return true;
  }


  //! Compute the joint angles from pose information
  bool DogBotControllerROSC::NextTrajectory(const SimpleQuadrupedPoseC &pose)
  {
    PoseAnglesC poseAngles;
    m_dogBotKinematics->Pose2Angles(pose,poseAngles,m_useVirtualJoints);
    return NextTrajectory(poseAngles);
  }

}


