//romea
#include "romea_localisation/Position2D.hpp"

//#include "../../tools/transformation/algorithms/SmartInverseRigidTransformation2D.hpp"

namespace romea {


//--------------------------------------------------------------------------
Position2D::Position2D():
  position(Eigen::Vector2d::Zero()),
  covariance(Eigen::Matrix2d::Zero())
{

}

//--------------------------------------------------------------------------
std::ostream & operator <<(std::ostream & os,const Position2D & position2d)
{
  os << " Position 2D :"<<std::endl;
  os << " position.x =" << position2d.position.x() << std::endl;
  os << " position.y =" << position2d.position.y() << std::endl;
  os << " covariance " <<std::endl;
  os << position2d.covariance << std::endl;
  return os;
}

//--------------------------------------------------------------------------
//Position2D Position2D::toBody(const Pose2D &referenceFramePose)const
//{


  //  SmartInverseRigidTransformation2D nedToBody(referenceFramePose.getPositionVector(),
  //                                              referenceFramePose.getOrientationAroundZDownAxis(),
  //                                              referenceFramePose.getCovariance());


  //  CartesianCoordinates2d bodyPosition;
  //  Eigen::Matrix2d bodyPositionCovariance;
  //  nedToBody(position_,covariance_,bodyPosition,bodyPositionCovariance);
  //  //TODO use std::move when eigen support will be available
  //  return BodyPosition2D(bodyPosition,bodyPositionCovariance);

//}

//--------------------------------------------------------------------------
Ellipse uncertaintyEllipse(const Position2D & position2d,
                           const double &sigmaScale)
{
  return Ellipse(position2d.position,
                 position2d.covariance,
                 sigmaScale);
}

}

