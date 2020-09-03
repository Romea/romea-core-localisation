//romea
#include "romea_localisation/PoseAndTwist2D.hpp"

namespace romea {

//--------------------------------------------------------------------------
PoseAndTwist2D::PoseAndTwist2D():
  pose(),
  twist()
{

}

//--------------------------------------------------------------------------
std::ostream & operator <<(std::ostream & os,const PoseAndTwist2D & poseAndBodyTwist2D)
{
  os << poseAndBodyTwist2D.pose;
  os << poseAndBodyTwist2D.twist;
  return os;
}

}

