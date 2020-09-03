//romea
#include "romea_localisation/Pose3D.hpp"
#include <romea_common/math/EulerAngles.hpp>
#include <romea_common/transform/SmartRotation3D.hpp>

namespace romea {

//--------------------------------------------------------------------------
Pose3D::Pose3D():
  position(Eigen::Vector3d::Zero()),
  orientation(Eigen::Vector3d::Zero()),
  covariance(Eigen::Matrix6d::Zero())
{

}

//--------------------------------------------------------------------------
void toPose2D(const Pose3D & pose3d, Pose2D & pose2d)
{
  pose2d.position.x()=pose3d.position.x();
  pose2d.position.y()=pose3d.position.y();
  pose2d.yaw=pose3d.orientation.z();
  pose2d.covariance = toSe2Covariance(pose3d.covariance);
}

//--------------------------------------------------------------------------
void toPosition3D(const Pose3D & pose3d, Position3D & position3d)
{
  position3d.position = pose3d.position;
  position3d.covariance = pose3d.covariance.block<3,3>(0,0);
}

//--------------------------------------------------------------------------
Pose2D toPose2D(const Pose3D & pose3d)
{
  Pose2D pose2d;
  toPose2D(pose3d,pose2d);
  return pose2d;
}

//--------------------------------------------------------------------------
Position3D toPosition3D(const Pose3D & pose3d)
{
  Position3D position3d;
  toPosition3D(pose3d,position3d);
  return position3d;
}

//--------------------------------------------------------------------------
Pose3D operator*(const Eigen::Affine3d & affine, const Pose3D & pose3D)
{
  SmartRotation3D smartRotation(pose3D.orientation);

  Eigen::Matrix3d R = affine.rotation();
  Eigen::Vector3d T = affine.translation();
  Eigen::Matrix3d rotation = affine.rotation()*smartRotation.R();

  Eigen::Matrix6d J =Eigen::Matrix6d::Zero();
  J.block<3,3>(0,0) = rotation;

  //derivative of rotation wrt angle around X = atan(r21/r22)
  double r21 = rotation(2,1);
  double r22 = rotation(2,2);
  double a21= r22/(r21*r21 + r22*r22);
  double a22= r21/(r21*r21 + r22*r22);
  J(3,3)=R.row(2).dot(a21*smartRotation.dRdAngleAroundXAxis().col(1)-a22*smartRotation.dRdAngleAroundXAxis().col(2));
  J(3,4)=R.row(2).dot(a21*smartRotation.dRdAngleAroundYAxis().col(1)-a22*smartRotation.dRdAngleAroundYAxis().col(2));
  J(3,5)=R.row(2).dot(a21*smartRotation.dRdAngleAroundZAxis().col(1)-a22*smartRotation.dRdAngleAroundZAxis().col(2));


  //derivative of rotation wrt angle around Y = - asin(r20)
  double r20 = rotation(2,0);
  double a20 = 1./(1- r20*r20);
  J(4,3)= R.row(2).dot(a20*smartRotation.dRdAngleAroundXAxis().col(0));
  J(4,4)= R.row(2).dot(a20*smartRotation.dRdAngleAroundYAxis().col(0));
  J(4,5)= R.row(2).dot(a20*smartRotation.dRdAngleAroundZAxis().col(0));


  //derivative of rotation wrt angle around Z = atan(r10/r00)
  double r10 = R(1,0);
  double r00 = R(0,0);
  double a10= r00 /(r00*r00 + r10*r10);
  double a00= r10 /(r00*r00 + r10*r10);

  J(5,3)=(-a00*rotation.row(0)+a10*rotation.row(1)).dot(smartRotation.dRdAngleAroundYAxis().col(0));
  J(5,4)=(-a00*rotation.row(0)+a10*rotation.row(1)).dot(smartRotation.dRdAngleAroundXAxis().col(0));
  J(5,5)=(-a00*rotation.row(0)+a10*rotation.row(1)).dot(smartRotation.dRdAngleAroundZAxis().col(0));

  Pose3D result;
  result.position=R*pose3D.position+T;
  result.orientation=rotation3DToEulerAngles(rotation);
  result.covariance=J*pose3D.covariance*J.transpose();
  return result;
}


//--------------------------------------------------------------------------
std::ostream & operator <<(std::ostream & os,const Pose3D & pose3D)
{
  os << "Pose 3D :"<< std::endl;
  os << " x =" << pose3D.position.x() << std::endl;
  os << " y =" << pose3D.position.y() << std::endl;
  os << " z =" << pose3D.position.z() << std::endl;
  os << " roll =" << pose3D.orientation.x() << std::endl;
  os << " pitch =" << pose3D.orientation.y() << std::endl;
  os << " yaw =" << pose3D.orientation.z() << std::endl;
  os << " covariance " << std::endl;
  os << pose3D.covariance << std::endl;

  return os;
}
}

