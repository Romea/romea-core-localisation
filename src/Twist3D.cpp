#include "romea_localisation/Twist3D.hpp"

namespace romea {

//--------------------------------------------------------------------------
Twist3D::Twist3D():
  linearSpeeds(Eigen::Vector3d::Zero()),
  angularSpeeds(Eigen::Vector3d::Zero()),
  covariance(Eigen::Matrix6d::Zero())
{

}

//--------------------------------------------------------------------------
Twist2D toTwist2D(const Twist3D & twist3d)
{
  Twist2D twist2d;
  toTwist2D(twist3d,twist2d);
  return twist2d;
}

//--------------------------------------------------------------------------
void toTwist2D(const Twist3D & twist3d, Twist2D & twist2d)
{
  twist2d.linearSpeeds.x()=twist3d.linearSpeeds.x();
  twist2d.linearSpeeds.y()=twist3d.linearSpeeds.y();
  twist2d.angularSpeed=twist3d.angularSpeeds.z();
  twist2d.covariance=toSe2Covariance(twist3d.covariance);
}

//--------------------------------------------------------------------------
std::ostream & operator <<(std::ostream & os, const Twist3D & twist3D)
{
  os << "Twist 3D :"<< std::endl;
  os << " linear.x =" << twist3D.linearSpeeds.x() <<std::endl;
  os << " linear.y =" << twist3D.linearSpeeds.y() <<std::endl;
  os << " linear.z =" << twist3D.linearSpeeds.z() <<std::endl;
  os << " angular.x =" << twist3D.angularSpeeds.x() << std::endl;
  os << " angular.y =" << twist3D.angularSpeeds.y() << std::endl;
  os << " angular.z =" << twist3D.angularSpeeds.z() << std::endl;
  os << " covariance " <<std::endl;
  os << twist3D.covariance << std::endl;

  return os;
}


}
