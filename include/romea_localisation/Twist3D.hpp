#ifndef romea_Twist3D_hpp
#define romea_Twist3D_hpp

//romea
#include "Twist2D.hpp"
#include <romea_common/math/Matrix.hpp>

namespace romea
{

struct Twist3D
{
  Twist3D();

  Eigen::Vector3d linearSpeeds;
  Eigen::Vector3d angularSpeeds;
  Eigen::Matrix6d covariance;
};

Twist2D toTwist2D(const Twist3D & twist3d);

void toTwist2D(const Twist3D & twist3d, Twist2D &twist2d);

std::ostream & operator <<(std::ostream & os,const Twist3D & twist3D);
}

#endif
