#ifndef romea_PoseAndTwist3D_hpp
#define romea_PoseAndTwist3D_hpp

//romea
#include "Pose3D.hpp"
#include "Twist3D.hpp"
#include "PoseAndTwist2D.hpp"

namespace romea {

struct PoseAndTwist3D
{
  PoseAndTwist3D();

  Pose3D pose;
  Twist3D twist;
};

PoseAndTwist2D toPoseAndTwist2D(const PoseAndTwist3D & poseAndTwist3D);

void toPoseAndTwist2D(const PoseAndTwist3D & poseAndTwist3D, PoseAndTwist2D & poseAndTwist2D );

std::ostream & operator <<(std::ostream & os,const PoseAndTwist3D & poseAndTwist3D);

}
#endif
