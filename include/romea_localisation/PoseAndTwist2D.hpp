#ifndef romea_PoseAndTwist2D_hpp
#define romea_PoseAndTwist2D_hpp

//romea
#include "Pose2D.hpp"
#include "Twist2D.hpp"

namespace romea {

struct PoseAndTwist2D
{
  PoseAndTwist2D();

  Pose2D pose;
  Twist2D twist;
};

std::ostream & operator <<(std::ostream & os,const PoseAndTwist2D & poseAndBodyTwist2D);

}

#endif

