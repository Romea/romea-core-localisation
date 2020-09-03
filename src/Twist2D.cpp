#include "romea_localisation/Twist2D.hpp"

namespace romea

{

//--------------------------------------------------------------------------
Twist2D::Twist2D():
  angularSpeed(0),
  linearSpeeds(Eigen::Vector2d::Zero()),
  covariance(Eigen::Matrix3d::Zero())
{

}

//--------------------------------------------------------------------------
std::ostream & operator <<(std::ostream & os,const Twist2D & twist2D)
{
  os << " Twist 2D :"<<std::endl;
  os << " linear.x =" << twist2D.linearSpeeds.x() << std::endl;
  os << " linear.y =" << twist2D.linearSpeeds.y() << std::endl;
  os << " angular.z =" << twist2D.angularSpeed << std::endl;
  os << " covariance " <<std::endl;
  os << twist2D.covariance << std::endl;
  return os;
}

}

