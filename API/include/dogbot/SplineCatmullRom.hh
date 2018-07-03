#ifndef DOGBOT_SPLINECATMULLROM_HEADER
#define DOGBOT_SPLINECATMULLROM_HEADER 1

#include <map>
#include <vector>
#include <eigen3/Eigen/Geometry>

namespace DogBotN {

  //! Control point for spline

  class SplinePoint3dC
  {
  public:
    SplinePoint3dC()
    {}

    SplinePoint3dC(float t,float x,float y,float z);

    SplinePoint3dC(float t,const Eigen::Vector3f &pnt)
     : m_timeDelta(t),
       m_point(pnt)
    {}

    float m_timeDelta = 1.0; // Time since last point
    Eigen::Vector3f m_point;
  };

  //! Spline evaluation
  // This spline wraps around forming a closed loop

  class SplineCatmullRom3dC
  {
  public:
    SplineCatmullRom3dC();

    //! Construct from a list of positions
    SplineCatmullRom3dC(std::vector<SplinePoint3dC> &points);

    //! Setup control points
    void Setup(const std::vector<SplinePoint3dC> &points);

    //! Evaluate position at time t
    bool Evaluate(float t,Eigen::Vector3f &pnt);

    //! Access total time
    float TotalTime() const
    { return m_totalTime; }

    std::map<float,SplinePoint3dC> &ControlPoints()
    { return m_trajectory; }

  protected:
    float m_totalTime = 0;
    std::map<float,SplinePoint3dC> m_trajectory;
  };


  class SplineLinear3dC
  {
  public:
    SplineLinear3dC();

    //! Construct from a list of positions
    SplineLinear3dC(std::vector<SplinePoint3dC> &points);

    //! Setup control points
    void Setup(const std::vector<SplinePoint3dC> &points);

    //! Evaluate position at time t
    bool Evaluate(float t,Eigen::Vector3f &pnt);

    //! Access total time
    float TotalTime() const
    { return m_totalTime; }

    std::map<float,SplinePoint3dC> &ControlPoints()
    { return m_trajectory; }
  protected:
    float m_totalTime = 0;
    std::map<float,SplinePoint3dC> m_trajectory;
  };

}

#endif
