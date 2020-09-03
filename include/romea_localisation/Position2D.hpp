#ifndef romea_Position2D_hpp
#define romea_Position2D_hpp

//romea
#include <romea_common/geometry/Ellipse.hpp>

namespace romea {

struct Position2D {

  Position2D();

  Eigen::Vector2d position;
  Eigen::Matrix2d covariance;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

};

Ellipse uncertaintyEllipse(const Position2D & position2d,
                           const double &sigmaScale);

std::ostream & operator <<(std::ostream & os,const Position2D & position2d);

}


#endif

