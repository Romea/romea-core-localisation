//romea
#include "romea_localisation/Pose2D.hpp"
#include "romea_localisation/Position2D.hpp"
#include <romea_common/math/EulerAngles.hpp>

namespace romea {

//--------------------------------------------------------------------------
Pose2D::Pose2D():
  yaw(0),
  position(Eigen::Vector2d::Zero()),
  covariance(Eigen::Matrix3d::Zero())
{

}

//--------------------------------------------------------------------------
Ellipse uncertaintyEllipse(const Pose2D & pose2d,
                           const double &sigmaScale)
{
  return Ellipse(pose2d.position,
                 pose2d.covariance.block<2,2>(0,0),
                 sigmaScale);
}

//--------------------------------------------------------------------------
std::ostream & operator <<(std::ostream & os,const Pose2D & pose2D)
{
  os << " Pose 2D :"<< std::endl;
  os << " x =" << pose2D.position.x() << std::endl;
  os << " y =" << pose2D.position.y() << std::endl;
  os << " yaw " << pose2D.yaw <<std::endl;
  os << " covariance " << std::endl;
  os << pose2D.covariance << std::endl;

  return os;
}


}

