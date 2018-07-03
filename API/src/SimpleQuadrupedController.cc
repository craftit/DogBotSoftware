
#include <cassert>
#include "dogbot/SimpleQuadrupedController.hh"

namespace DogBotN {

  SimpleQuadrupedControllerC::SimpleQuadrupedControllerC()
  {}

  //! Virtual destructor
  SimpleQuadrupedControllerC::~SimpleQuadrupedControllerC()
  {}

  //! Set omega
  void SimpleQuadrupedControllerC::SetOmega(float omega)
  {
    m_omega = omega;
  }

  //! Set the gait style
  bool SimpleQuadrupedControllerC::SetStyle(const std::string &styleName)
  {
    return false;
  }

  //! Do a single timestep
  bool SimpleQuadrupedControllerC::Step(float timeStep,SimpleQuadrupedPoseC &positions)
  {
    return true;
  }

}
