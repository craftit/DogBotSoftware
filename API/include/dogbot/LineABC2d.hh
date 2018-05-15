// This file is part of RAVL, Recognition And Vision Library
// Copyright (C) 2001, University of Surrey
// This code may be redistributed under the terms of the GNU Lesser
// General Public License (LGPL). See the lgpl.licence file for details or
// see http://www.gnu.org/copyleft/lesser.html
// file-header-ends-here
#ifndef DOGBOT_LINE2ABC_HEADER
#define DOGBOT_LINE2ABC_HEADER 1
//! author="Radek Marik"
//! date="26.06.1994"

#include <eigen3/Eigen/Geometry>

namespace DogBotN {


  inline bool IsAlmostZero(float num,float tolerance = 1e-8)
  { return fabsf(num) < tolerance; }

  //: Generate a vector perpendicular to v.
  inline Eigen::Vector2f Perpendicular(const Eigen::Vector2f &v)
  { return Eigen::Vector2f(-v[1],v[0]); }

  //: Returns the third coordinate of the cross product of this vector
  //: and the vector 'v'.
  inline float Cross(const Eigen::Vector2f &data,const Eigen::Vector2f & vector)
  { return data[0] * vector[1] - data[1] * vector[0]; }

  //: Line in 2D space - equation Ax+By+C = 0
  // The class LineABC2dC represents a line embedded in the 2D plane.
  // The line is represented by the equation aa*x+bb*y+cc = 0.

  class LineABC2dC {
  public:
    typedef float RealT;

    //! Creates the non-existing line (0,0,0).
    LineABC2dC()
      : normal(0.0, 0.0), d(0.0)
    {}

    //! Creates the line determined by the equation a*x+b*y+c = 0.
    LineABC2dC(RealT a, RealT b, RealT c)
      : normal(a, b), d(c)
    {}

    //! Creates the line determined by the equation a*x+b*y+c = 0.
    LineABC2dC(const Eigen::Vector2f &normal, RealT c)
      : normal(normal), d(c)
    {}

    //! Create a line from a normal and a point on the line.
    static LineABC2dC CreateFromNormalAndPoint(const Eigen::Vector2f &normal,const Eigen::Vector2f &point)
    { return LineABC2dC(normal,-normal.dot(point)); }

    //! Create a line from two points.
    static LineABC2dC CreateFromPointToPoint(const Eigen::Vector2f &start,const Eigen::Vector2f &end)
    {
      Eigen::Vector2f normal = Perpendicular(end-start);
      return LineABC2dC(normal,-normal.dot(start));
    }

    //! Create a line from a point on the line and a direction
    static LineABC2dC CreateFromPointAndDirection(const Eigen::Vector2f &point,const Eigen::Vector2f &direction)
    {
      Eigen::Vector2f normal = Perpendicular(direction);
      return LineABC2dC(normal,-normal.dot(point));
    }

    //! Returns a vector normal to the line.
    Eigen::Vector2f Normal() const
    { return normal; }

    //! Returns the normal of the line normalised to have unit size.
    Eigen::Vector2f UnitNormal() const
    { return normal / normal.norm(); }

    //! Returns the distance of the line from the origin of the coordinate
    //! system.
    RealT Rho() const
    { return d / normal.norm(); }

    //! Returns parameter a.
    RealT A() const
    { return normal[0]; }

    RealT B() const
    { return normal[1]; }
    //: Returns parameter b.

    RealT C() const
    { return d; }
    //: Returns parameter c.

    RealT ValueX(const RealT y) const
    { return (A() == 0) ? (RealT) 0 : (-B()*y - C()) / A(); }
    //: Returns the value of x coordinate if the y coordinate is known.
    // If the parameter A() is zero, the zero is returned.

    RealT ValueY(const RealT x) const
    { return (B() == 0) ? (RealT) 0 : (-A()*x - C()) / B(); }
    //: Returns the value of y coordinate if the x coordinate is known.
    // If the parameter B() is zero, the zero is returned.

    //:--------------------------
    //: Geometric constructions.

    RealT Residuum(const Eigen::Vector2f & p) const
    { return normal[0] * p[0] + normal[1] * p[1] + d; }
    //: Returns the value of the function A()*p[0]+B()*p[1]+C() often
    //: used in geometric computations.

    LineABC2dC & MakeUnitNormal();
    //: Normalises the equation so that the normal vector is unit.

    bool AreParallel(const LineABC2dC & line) const;
    //: Returns true if the lines are parallel.

    bool Intersection(const LineABC2dC & line,Eigen::Vector2f &here) const;
    //: Find the intersection of two lines.
    // If the intersection doesn't exist, the function returns false.
    // The intersection is assigned to 'here'.

    RealT SqrEuclidDistance(const Eigen::Vector2f & point) const;
    //: Returns the squared Euclidean distance of the 'point' from the line.

    RealT SignedDistance(const Eigen::Vector2f & point) const
    { return Residuum(point) / normal.norm(); }
    //: Returns the signed distance of the 'point' from the line.
    // The return value is greater than 0 if the point is on the left
    // side of the line. The left side of the line is determined
    // by the direction of the normal.

    RealT Distance(const Eigen::Vector2f & point) const
    { return fabsf(SignedDistance(point)); }
    //: Returns the distance of the 'point' from the line.

    Eigen::Vector2f Projection(const Eigen::Vector2f & point) const
    { return point - normal *(Residuum(point)/normal.squaredNorm()); }
    //: Returns the point which is the orthogonal projection of the 'point'
    //: to the line.
    // It is the same as intersection of this line with
    // the perpendicular line passing through the 'point'.

  private:
    Eigen::Vector2f normal;
    // The normal of the line.

    RealT     d;
    // The distance of the line from the origin of the coordinate system
    // multiplied by the size of the normal vector of the line.
  };


}
#endif
