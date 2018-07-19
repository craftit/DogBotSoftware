

%module pydogbotapi

%include <std_shared_ptr.i>
%include <std_string.i>

%{
#include "dogbot/DogBotAPI.hh"
#include "dogbot/LegController.hh"
#include "dogbot/DogBotController.hh"
#include "dogbot/PoseAngles.hh"

namespace DogBotN {

  std::shared_ptr<DogBotAPIC> OpenAPI(const std::string &robotName = "",const std::string &connection ="local")
  {
    return std::make_shared<DogBotAPIC>(connection,DogBotAPIC::DefaultConfigFile(robotName));
  }

}

%}

%shared_ptr(DogBotN::JointC)
%shared_ptr(DogBotN::DogBotAPIC)
%shared_ptr(DogBotN::LegControllerC)
%shared_ptr(DogBotN::DogBotControllerC)

namespace DogBotN {

//! Interface for a single joint

class JointC 
{
  public:    
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

//! Servo level API

class DogBotAPIC 
{
  public:
    //! Get a joint by name
    std::shared_ptr<JointC> GetJointByName(const std::string &name);
    
    //! Access a list of joints
    std::vector<std::shared_ptr<JointC> > ListJoints();

    //! Get the current time
    static double TimeNow();

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
    
    //! Make the robot go limp by disabling all motors
    void MotorsOffAll();

    //! Switch the break on for all motors
    void BrakeAll();

    //! Initiate an emergency stop
    void EmergencyStop();
};

//! Open connection to robot.
std::shared_ptr<DogBotAPIC> OpenAPI(const std::string &robotName = "",const std::string &connection = "local");

//! Class to manage the positioning of a single leg.

class LegControllerC
{
public:
  //! Create a leg controller
  LegControllerC(std::shared_ptr<DogBotAPIC> &api,const std::string &legName,bool useVirtualKnee);

  //! Goto a position in the leg coordinate frame
  //! Returns true if the requested position is reachable
  bool Goto(float x,float y,float z,float torqueLimit);

  //! Goto a joint angles
  bool GotoJointAngles(float roll,float pitch,float knee,float torqueLimit);
  
  //! Get current joint states
  bool GetJointStates(double theTime,
                    float &angleRoll,float &anglePitch,float &angleknee,
                    float &velocityRoll,float &velocityPitch,float &velocityKnee,
                    float &torqueRoll,float &torquePitch,float &torqueknee
                    );

  //! Compute an estimate of the force on a foot and where it is.
  bool ComputeFootForce(double atTime,float &positionX,float &positionY,float &positionZ,float &forceX,float &forceY,float &forceZ);

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

  //! Set angle and optionally torque for a particular joint
  void SetJoint(int jnt,float pos,float torque = 0);

  //! Access joint information
  float JointPosition(int jnt) const;

  //! Access joint information
  float JointTorque(int jnt) const;

};

//! Robot pose stored as foot positions in the robot coordinate system

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

  //! Compute the joint angles from pose information
  // If the angle computation fails, this will use the closest reachable point to one requested.
  // Returns false if the updates succeeded for all servos.
  bool NextTrajectory(const SimpleQuadrupedPoseC &pose);

};

}