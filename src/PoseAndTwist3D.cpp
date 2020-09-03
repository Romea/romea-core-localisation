#include "romea_localisation/PoseAndTwist3D.hpp"

namespace romea
{

//--------------------------------------------------------------------------
PoseAndTwist3D::PoseAndTwist3D():
  pose(),
  twist()
{

}

//--------------------------------------------------------------------------
PoseAndTwist2D toPoseAndTwist2D(const PoseAndTwist3D & poseAndTwist3D)
{
  PoseAndTwist2D poseAndTwist2D;
  toPoseAndTwist2D(poseAndTwist3D,poseAndTwist2D);
  return poseAndTwist2D;
}

//--------------------------------------------------------------------------
void toPoseAndTwist2D(const PoseAndTwist3D & poseAndTwist3D,
                      PoseAndTwist2D & poseAndTwist2D)
{
  toPose2D(poseAndTwist3D.pose,poseAndTwist2D.pose);
  toTwist2D(poseAndTwist3D.twist,poseAndTwist2D.twist);
}

//--------------------------------------------------------------------------
std::ostream & operator <<(std::ostream & os,const PoseAndTwist3D & poseAndTwist3D)
{
  os << poseAndTwist3D.pose;
  os << poseAndTwist3D.twist;
  return os;
}


}
