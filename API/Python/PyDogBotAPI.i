

#ifdef SWIG
%module DogBotAPI
#endif

%include <std_shared_ptr.i>

%{
#include "dogbot/DogBotAPI.hh"
#include "dogbot/LegController.hh"
%}

namespace DogBotN {


class JointC 
{
  public:
  
    //! Goto a given angle
    virtual bool DemandPosition(float position,float torqueLimit);

    //! Setup a trajectory, with an update rate and torque limit
    virtual bool SetupTrajectory(float updatePeriod,float torqueLimit);

    //! Provide a point on a trajectory with a given position
    //! SetupTrajectory should be called before using this function,
    //! and it must be called every 'updatePeriod' seconds.
    //! Setting the expected torque is optional, and may be left at 0 if unknown. 
    virtual bool DemandTrajectory(float position,float torque = 0);
  
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

    std::shared_ptr<JointC> GetJointByName(const std::string &name);
    
    std::vector<std::shared_ptr<JointC> > ListJoints();

    //! Access an ordered list of the four leg names
    static const std::vector<std::string> &LegNames();

    //! Access names of leg joints.
    static const std::vector<std::string> &LegJointNames();

};

}