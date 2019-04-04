#pragma once

#include <jsoncpp/json/json.h>
#include <cassert>
#include <eigen3/Eigen/Geometry>

namespace DogBotN {


  //! This computes the position of the end effector given the end effector

  // l1 = Upper leg length.
  // l2 = Lower leg length

  // Coordinates:
  //  x = sideways
  //  y = forward/back
  //  z = Height above ground
  //
  // Angles:
  //  0 - Roll
  //  1 - Pitch
  //  2 - Knee
  //
  // In the 4 bar linkage code the following variables are used:
  //   theta - Servo position
  //   psi - Joint angle

  class LegKinematicsV4C
  {
  public:
    //! Default constructor
    LegKinematicsV4C();

    //! Construct from a json object
    LegKinematicsV4C(const Json::Value &value);

    //! Create
    LegKinematicsV4C(float l1,float l2);

    //! Access name of leg.
    const std::string &Name() const
    { return m_name; }

    //! Set name of leg.
    void SetName(const std::string &name)
    { m_name = name; }

    //! Configure from JSON
    bool ConfigureFromJSON(const Json::Value &value);

    //! Get the servo configuration as JSON
    Json::Value ConfigAsJSON() const;

    //! Inverse kinematics for the leg using a virtual joint for the knee
    //! Compute joint angles needed to get to a 3d position in a leg coordinate system
    //! Return true if position is reachable
    bool InverseVirtual(const Eigen::Vector3f &position, Eigen::Vector3f &angles) const;

    //! Forward kinematics for the leg using a virtual joint for the knee
    //! Compute the position of the foot relative to the top of the leg from the joint angles.
    bool ForwardVirtual(const Eigen::Vector3f &angles,Eigen::Vector3f &position) const;

    //! Inverse kinematics for the leg
    //! Compute joint angles needed to get to a 3d position in a leg coordinate system
    //! Return true if position is reachable
    bool InverseDirect(const Eigen::Vector3f &position,Eigen::Vector3f &angles) const;

    //! Forward kinematics for the leg
    //! Compute the position of the foot relative to the top of the leg from the joint angles.
    bool ForwardDirect(const Eigen::Vector3f &angles,Eigen::Vector3f &position) const;

    //! Compute an estimate of the force on a foot and where it is given some angles and torques
    bool ComputeFootForce(
        const Eigen::Vector3f &angles,
        const Eigen::Vector3f &jointVelocities,
        const Eigen::Vector3f &torques,
        Eigen::Vector3f &position,
        Eigen::Vector3f &velocity,
        Eigen::Vector3f &force
        ) const;


    //! Use alternate solution ?
    bool UseAlternateSolution() const
    { return m_alternateSolution; }

    // ! Access joint directions
    float JointDirection(int jnt) const
    {
      assert(jnt >= 0 && jnt < 3);
      return m_jointDirections[jnt];
    }

    // ! Access joint directions as vector
    const Eigen::Vector3f &JointDirections() const
    { return m_jointDirections; }

    //! Length of upper leg
    float LengthUpperLeg() const
    { return m_l1; }

    //! Length of lower leg
    float LengthLowerLeg() const
    { return m_l2; }

    //! Offset from roll axis of rotation
    float LengthZDrop() const
    { return 0; }

    //! Distance bellow the pivot of the centre of the foot
    float LengthFootDrop() const
    { return m_footDrop; }

    //! Radius of foot sphere
    float FootSphereRadius() const
    { return m_footSphereRadius; }

    //! Leg origin.
    float LegOrigin(int coordinate) const
    { return m_legOrigin[coordinate]; }

    //! Leg origin.
    const Eigen::Vector3f &LegOrigin() const
    { return m_legOrigin; }

    //! Compute the maximum extension of the leg vertically down
    float MaxExtension() const;

    //! Compute the minimum extension of the leg vertically down
    float MinExtension() const;

    //! Compute the maximum stride length at a given leg extension.
    float StrideLength(float extension) const;

  protected:
    void Init();

    std::string m_name; // Leg name

    float m_minExtension = -1;
    float m_maxExtension = -1;
    // These are used to compute the leg positions, though they don't really belong here.
    float m_bodyWidth = 0.304;
    float m_bodyLength = 0.556;

    Eigen::Vector3f m_legOrigin = { 0, 0, 0};
    Eigen::Vector3f m_jointDirections = { 1.0, 1.0, 1.0 };

    float m_l0 = 0.09875; // Hip offset
    float m_l1 = 0.315; // Upper leg length
    float m_l2 = 0.30;  // Lower leg length

    float m_footDrop = 0.025; // Distance of centre of foot bellow
    float m_footSphereRadius = 0.053; // Radius of foot sphere

    bool m_alternateSolution = false;

  };

}

