

%module DogBotAPI

%include <std_shared_ptr.i>

%{
#include "dogbot/DogBotAPI.hh"
#include "dogbot/LegController.hh"
#include "dogbot/DogBotController.hh"
#include "dogbot/PoseAngles.hh"
%}

%shared_ptr(DogBotN::JointC)
%shared_ptr(DogBotN::DogBotAPIC)
%shared_ptr(DogBotN::LegControllerC)
%shared_ptr(DogBotN::DogBotControllerC)

namespace DogBotN {


class JointC 
{
  public:
    JointC();
    virtual ~JointC();
    
    //! Setup a trajectory, with an update rate and torque limit
    bool SetupTrajectory(float updatePeriod,float torqueLimit);

    //! Provide a point on a trajectory with a given position
    //! SetupTrajectory should be called before using this function,
    //! and it must be called every 'updatePeriod' seconds.
    //! Setting the expected torque is optional, and may be left at 0 if unknown. 
    bool DemandTrajectory(float position,float torque = 0);

    //! Goto a given angle
    bool DemandPosition(float position,float torqueLimit);
  
    //! Get last reported state of the servo and the time it was taken in seconds.
    bool GetState(double &theTime,double &position,double &velocity,double &torque) const;

    //! Estimate state at the given time.
    //! This will linearly extrapolate position, and assume velocity and torque are
    //! the same as the last reading.
    //! If the data is more than 5 ticks away from the current time the method returns false.
    bool GetStateAt(double theTime,double &position,double &velocity,double &torque) const;

    //! Access update tick duration
    //! This is the expected time between joint updates.
    //! This is set to -1 if the duration is not yet known.
    virtual double TickDuration() const;

    //! Last reported position
    float Position() const;

    //! Last reported torque
    float Torque() const;

    //! Last reported velocity
    float Velocity() const;
};

class DogBotAPIC 
{
  public:
    DogBotAPIC();
    ~DogBotAPIC();

    //! Get a joint by name
    std::shared_ptr<JointC> GetJointByName(const std::string &name);
    
    //! Access a list of joints
    std::vector<std::shared_ptr<JointC> > ListJoints();

    //! Access an ordered list of the four leg names
    static const std::vector<std::string> &LegNames();

    //! Access names of leg joints.
    static const std::vector<std::string> &LegJointNames();

    //! Home all the joints in the robot
    //! Returns true if succeeded
    bool HomeAll();

    //! Tell all servos to hold the current position
    void DemandHoldPosition();

    //! Reset all controllers.
    void PowerOnAll();

    //! Request all controllers go into low power mode
    void StandbyAll();
};


//! Class to manage the positioning of a single leg.

class LegControllerC
{
public:

  LegControllerC(std::shared_ptr<DogBotAPIC> &api,const std::string &legName,bool useVirtualKnee);

  //! Goto a position in the leg coordinate frame
  //! Returns true if the requested position is reachable
  bool Goto(float x,float y,float z,float torqueLimit);

  //! Goto a joint angles
  bool GotoJointAngles(float roll,float pitch,float knee,float torqueLimit);
  
};

//! Robot pose in terms of joint angles and expected torques.

class PoseAnglesC
{
public:
  PoseAnglesC();

  // Identify joint in array.
  // Leg: as DogBotAPIC::LegNames()
  // legJnt: 0=Roll, 1=Pitch, 2=Knee
  static int JointId(int leg,int legJnt);

  //! Name of joint for the given leg and joint number.
  static std::string JointName(int leg,int legJnt);

  //! Set leg joint angles
  void SetLegJointAngles(int leg,const Eigen::Vector3f &vec);

  //! Set angle for a particular joint
  void SetJoint(int jnt,float pos,float torque);

  //! Access joint information
  float JointPosition(int jnt) const;

  //! Access joint information
  float JointTorque(int jnt) const;

protected:
  JointAngleC m_joints[12];
};

class SimpleQuadrupedPoseC
{
public:
  SimpleQuadrupedPoseC();
  
  // Set the leg goal position
  void SetLegPosition(int legId,float x,float y,float z);

  // Get the leg goal position
  void GetLegPosition(int legId,float &x,float &y,float &z) const;
};


//! Class to manage the position of all the leg joints on the robot.

class DogBotControllerC
{
public:
  DogBotControllerC(std::shared_ptr<DogBotAPIC> &api);

  //! Setup trajectory
  bool SetupTrajectory(float updatePeriod,float torqueLimit);

  //! Send next trajectory position, this should be called at 'updatePeriod' intervals as setup
  //! with SetupTrajectory.
  bool NextTrajectory(const PoseAnglesC &pose);

};

}