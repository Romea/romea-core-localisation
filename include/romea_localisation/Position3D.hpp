#ifndef romea_Position3D_hpp
#define romea_Position3D_hpp

//Eigen
#include <Eigen/Core>

namespace romea {

struct Position3D {

  Position3D();

  Eigen::Vector3d position;
  Eigen::Matrix3d covariance;

};

}


#endif

