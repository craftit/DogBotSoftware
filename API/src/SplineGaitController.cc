
#include "dogbot/SplineGaitController.hh"

#include <fstream>

namespace DogBotN {

  FootTrajectoryC::FootTrajectoryC()
  {

  }

  //! Setup trajectory
  FootTrajectoryC::FootTrajectoryC(float Zc,float Xc,float Lpr,float tpr,float Had,float tad,float tpu)
    : m_Zc(Zc),
      m_Xc(Xc),
      m_Lpr(Lpr),
      m_tpu(tpu),
      m_Had(Had),
      m_tad(tad),
      m_tpr(tpr)
  {}


  std::vector<SplinePoint3dC> FootTrajectoryC::GenerateTrajectory(float rz,float xoff,float zoff) const
  {
    std::vector<SplinePoint3dC> pnts;
    pnts.reserve(8);

    float Lpu = cos(m_apu) * m_Rpu;
    float Zpu = sin(m_apu) * m_Rpu;

    float LpuFin = 1.333 * Lpu;
    float Sl = LpuFin + m_Lpr; // Total stride length
    float Oad = -Sl / 7.0; // Was 9. what fraction of distance to move the half the adjustment vertical distance
    float nXc = m_Xc - Sl/2.0;

    pnts.push_back(SplinePoint3dC(m_tpr         ,nXc + m_Lpr          ,xoff ,-(zoff + m_Zc)));        // 1
    if(m_Rpu > 0) {

      //float timePush = m_Lpr
      pnts.push_back(SplinePoint3dC(2.0*m_tpu/3.0 ,nXc                ,xoff ,-(zoff + m_Zc)));      // 2
      pnts.push_back(SplinePoint3dC(m_tpu / 3.0   ,nXc - Lpu          ,xoff ,-(zoff + m_Zc-Zpu)));  // 3
    }

    pnts.push_back(SplinePoint3dC(m_tad/4.0     ,nXc - LpuFin         ,xoff ,-(zoff + m_Zc)));        // 4
    pnts.push_back(SplinePoint3dC(m_tad/4.0     ,nXc - LpuFin + Oad   ,xoff ,-(zoff + m_Zc+m_Had/2.0)));// 5
    pnts.push_back(SplinePoint3dC(m_tad/4.0     ,nXc + m_Lpr - Sl/2.0 ,xoff ,-(zoff + m_Zc+m_Had)));    // 6
    pnts.push_back(SplinePoint3dC(m_tad/4.0     ,nXc + m_Lpr - Oad    ,xoff ,-(zoff + m_Zc+m_Had/2.0)));// 7

    for(auto &a : pnts) {
      Eigen::Vector3f op = a.m_point;
      a.m_point[0] = op[0] * cos(rz) - op[1] * sin(rz);
      a.m_point[1] = op[0] * sin(rz) + op[1] * cos(rz);
      a.m_point[2] = op[2];
    }

#if 0
    // Resample
    {
      SplineLinear3dC linear(pnts);

      std::vector<SplinePoint3dC> newPnts;
      int num = 100;
      float timeInc = linear.TotalTime()/num;
      float t = 0;
      for(int i = 0;i < num;i++,t+=timeInc) {
        Eigen::Vector3f pnt;
        linear.Evaluate(t,pnt);
        newPnts.push_back(SplinePoint3dC(t,pnt));
      }
      pnts = newPnts;
    }
#endif

    return pnts;
  }

  // ----------------------------------------------------------

  SplineGaitControllerC::SplineGaitControllerC()
   : m_footTrajectories(4)
  {
    GenerateFootTrajectory(SGT_Walk);
  }

  void SplineGaitControllerC::GenerateFootTrajectory(enum SplineGaitTypeT gaitType)
  {
    float hightAdjust = 0.2;
    float timePush = 0.02;

    switch(gaitType)
    {
      case SGT_Walk: {
        // Walk
        m_lengthPropel = 0.25;
        m_timePropel = 0.75;
        hightAdjust = 0.15;
        timePush = 0.02;

        const int LegFL=0; // 1
        const int LegBL=2; // 2
        const int LegFR=1; // 3
        const int LegBR=3; // 4

        m_phases[LegFL] = 0;
        m_phases[LegBR] = 3* M_PI/2.0;
        m_phases[LegFR] = M_PI;
        m_phases[LegBL] = M_PI/2.0;

        //m_omega = 1;
      } break;
      case SGT_Trot: {
        // Trot
        m_timePropel = 0.5;
        m_lengthPropel = 0.15;

        m_phases[0] = 0;
        m_phases[1] = M_PI;
        m_phases[2] = M_PI;
        m_phases[3] = 0;

        //m_omega = 6;

      } break;
      default:
        std::cerr << "Unrecognised gait type, ignoring. " << std::endl;
        return ;
    }

    FootTrajectoryC trajectory(
        m_zCentre, // Zc,  Z center
        m_Xcentre, // Xc,  X center
        m_lengthPropel,  // Lpr,
        m_timePropel,   // tpr,
        m_hightAdjust,   // tad, Time adjust
        hightAdjust,   // Had, hight adjust
        timePush    // Tpu, Time push
        );

    m_footTrajectories[0].Setup(trajectory.GenerateTrajectory(-m_footRotate,-m_footSeperation,m_tiltX + m_tiltY));
    m_footTrajectories[1].Setup(trajectory.GenerateTrajectory(-m_footRotate,m_footSeperation,m_tiltX - m_tiltY));
    m_footTrajectories[2].Setup(trajectory.GenerateTrajectory(m_footRotate,-m_footSeperation,-m_tiltX + m_tiltY));
    m_footTrajectories[3].Setup(trajectory.GenerateTrajectory(m_footRotate,m_footSeperation,-m_tiltX - m_tiltY));

  }


  //! Set the gait style
  bool SplineGaitControllerC::SetStyle(const std::string &styleName)
  {
    std::cerr << "Updating gait style: '" << styleName << "' " << std::endl;
    if(styleName == "walk") {
      GenerateFootTrajectory(SGT_Walk);
      return true;
    }
    if(styleName == "trot") {
      GenerateFootTrajectory(SGT_Trot);
      return true;
    }
    std::cerr << "Unknown gait style: '" << styleName << "' " << std::endl;
    return false;
  }

  void SplineGaitControllerC::PlotGait() {

    int numPnts = 100;

    {
      std::ofstream plot("points.csv");
      plot << "T,X,Y,Z" << std::endl;

      auto &pmap = m_footTrajectories[0].ControlPoints();
      for(auto it = pmap.begin();it != pmap.end();++it) {
        Eigen::Vector3f pnt = it->second.m_point;
        plot << it->first << "," << pnt[0] << "," << pnt[1] << "," << pnt[2] << std::endl;
      }
    }

    {
      std::ofstream plot("gait.csv");
      plot << "T,X,Y,Z" << std::endl;

      float totalTime = m_footTrajectories[0].TotalTime();
      float timeStep = totalTime/numPnts;

      for(float t = 0;t < totalTime;t+= timeStep) {
        Eigen::Vector3f pnt;
        m_footTrajectories[0].Evaluate(t,pnt);
        plot << t << "," << pnt[0] << ","  << pnt[1] << "," << pnt[2] << std::endl;
      }
    }

  }

  //! Do a single timestep
  bool SplineGaitControllerC::Step(float timeStep,SimpleQuadrupedPoseC &pose)
  {
    m_phase += m_omega * timeStep;
    if(m_phase > M_PI*2)
      m_phase -= M_PI*2;
    if(m_phase < 0)
      m_phase += M_PI*2;


    for(int i = 0;i < 4;i++) {
      float phase = m_phase + m_phases[i];
      if(m_phase > M_PI*2)
        m_phase -= M_PI*2;
      if(m_phase < 0)
        m_phase += M_PI*2;

      float t = phase * m_footTrajectories[i].TotalTime() / (M_PI*2.0);

      Eigen::Vector3f pnt;
      Eigen::Vector3f angles;
      m_footTrajectories[i].Evaluate(t,pnt);

      Eigen::Vector3f pntx(
          pnt[1],
          pnt[0],
          -m_zOffset - pnt[2]
          );

#if 0
      if(!m_legKinematics.Inverse(pntx,angles)) {
        //RavlError("Kinematics failed.");
      }
#endif

      pose.SetLegPosition(i,pntx[0],pntx[1],pntx[2]);
    }

    return true;
  }

}
