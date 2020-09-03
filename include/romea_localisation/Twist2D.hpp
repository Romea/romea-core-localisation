#ifndef romea_Twist2D_hpp
#define romea_Twist2D_hpp

//Eigen
#include <Eigen/Core>

namespace romea
{

struct Twist2D
{
  Twist2D();

  double angularSpeed;
  Eigen::Vector2d linearSpeeds;
  Eigen::Matrix3d covariance;
};

std::ostream & operator <<(std::ostream & os,const Twist2D & twist2D);
}

#endif
