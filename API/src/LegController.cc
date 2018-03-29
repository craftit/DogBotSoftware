

#include "dogbot/LegController.hh"
#include "dogbot/Util.hh"

namespace DogBotN {

  //! Class to manage the positioning of a single leg.

  LegControllerC::LegControllerC(std::shared_ptr<DogBotAPIC> &api,const std::string &legName)
   : m_legName(legName),
     m_api(api)
  {
    Init();
  }


  bool LegControllerC::Init()
  {
    m_kinematics = m_api->LegKinematicsByName(m_legName);
    if(!m_kinematics) {
      m_log->error("Failed to find leg '{}' ",m_legName);
      return false;
    }

    m_legJointNames.clear();
    m_legJointNames.push_back(m_legName + "_roll");
    m_legJointNames.push_back(m_legName + "_pitch");
    m_legJointNames.push_back("virtual_" + m_legName + "_knee");

    for(int i = 0;i < 3;i++) {
      m_joints[i] = m_api->GetJointByName(m_legJointNames[i]);
      if(!m_joints[i]) {
        m_log->error("Failed to find {} joint. ",m_legJointNames[i]);
        return false;
      }
    }

    return true;
  }

  //! Goto a position
  //! Returns true position is reachable
  bool LegControllerC::Goto(float x,float y,float z,float torque)
  {
    float at[3] = {x,y,z};
    float angles[3];
    if(!m_kinematics->Inverse(at,angles)) {
      m_log->warn("Failed to find solution.");
      return false;
    }

    m_log->info("Setting angles to {} {} {}  for  {} {} {} ",
                   DogBotN::Rad2Deg(angles[0]),DogBotN::Rad2Deg(angles[1]),DogBotN::Rad2Deg(angles[2]),
                   at[0],at[1],at[2]);

    // FIXME:- If move fails what should we do ?

    if(!m_joints[0]->DemandPosition(angles[0],torque))
      return false;
    if(!m_joints[1]->DemandPosition(angles[1],torque))
      return false;
    if(!m_joints[2]->DemandPosition(angles[2],torque))
      return false;

    return true;
  }


}


