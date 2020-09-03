#ifndef romea_Pose3D_hpp
#define romea_Pose3D_hpp

// Eigen
#include <Eigen/Geometry>

//romea
#include "Pose2D.hpp"
#include "Position3D.hpp"
#include <romea_common/math/Matrix.hpp>

namespace romea {

struct Pose3D {

  Pose3D();

  Eigen::Vector3d position;
  Eigen::Vector3d orientation;
  Eigen::Matrix6d covariance;

};

Pose2D toPose2D(const Pose3D & pose3d);

Position3D toPosition3D(const Pose3D & pose3d);

void toPose2D(const Pose3D & pose3d, Pose2D & pose2d);

void toPosition3D(const Pose3D & pose3d, Position3D & position3d);

std::ostream & operator <<(std::ostream & os,const Pose3D & pose3D);

Pose3D operator*(const Eigen::Affine3d & affine, const Pose3D & pose3d);

}


#endif

