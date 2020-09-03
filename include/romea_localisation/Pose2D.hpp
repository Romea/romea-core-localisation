#ifndef romea_Pose_hpp
#define romea_Pose_hpp

//romea
#include <romea_common/geometry/Ellipse.hpp>

namespace romea {

struct Pose2D {

  Pose2D();

  double yaw;
  Eigen::Vector2d position;
  Eigen::Matrix3d covariance;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

};

Ellipse uncertaintyEllipse(const Pose2D & pose2d,
                           const double &sigmaScale);

std::ostream & operator <<(std::ostream & os,const Pose2D & pose2D);

}


#endif

