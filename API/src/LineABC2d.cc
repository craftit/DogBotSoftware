
#include "dogbot/LineABC2d.hh"

namespace DogBotN {

  LineABC2dC & LineABC2dC::MakeUnitNormal() {
    RealT size = normal.norm();
    normal /= size;
    d      /= size;
    return *this;
  }

  bool LineABC2dC::AreParallel(const LineABC2dC & line) const {
    RealT crossSize = Cross(Normal(),line.Normal());
    return  IsAlmostZero(crossSize);
  }

  bool LineABC2dC::Intersection(const LineABC2dC & line,Eigen::Vector2f &here) const  {
    RealT crossSize = Cross(Normal(),line.Normal());
    if ( IsAlmostZero(crossSize) )
      return false;
    here = Eigen::Vector2f((line.C()*B() - line.B()*C())/crossSize,
                    (line.A()*C() - line.C()*A())/crossSize);
    return true;
  }

  LineABC2dC::RealT LineABC2dC::SqrEuclidDistance(const Eigen::Vector2f & point) const {
    RealT t = Residuum(point);
    return t*t/normal.squaredNorm();
  }
}
