#ifndef _R2WLocalisationResults_hpp
#define _R2WLocalisationResults_hpp


//romea
#include <romea_common/time/Time.hpp>
#include "../PoseAndTwist3D.hpp"

namespace romea {

template <class State>
class R2WLocalisationResults : public State
{

public :

  template <typename... Args>
  R2WLocalisationResults(Args... args):
    State(std::forward<Args>(args)...),
    duration_(Duration::zero())
  {

  }

  virtual ~R2WLocalisationResults()=default;

  virtual void setDuration(const Duration & duration)
  {
    duration_=duration;
  }

  virtual const double & getX() const=0;
  virtual const double & getY() const=0;
  virtual const double & getYaw() const=0;
  virtual const double & getYawVariance() const=0;

  virtual Eigen::Vector3d getPose() const=0;
  virtual Eigen::Matrix3d getPoseCovariance() const=0;

  virtual const double & getLinearSpeed() const=0;
  virtual const double & getLateralSpeed() const=0;
  virtual const double & getAngularSpeed() const=0;

  virtual Eigen::Vector3d getTwist() const=0;
  virtual Eigen::Matrix3d getTwistCovariance() const=0;

  virtual Pose2D toPose2D() const =0;
  virtual Pose3D toPose3D() const=0;

  virtual PoseAndTwist2D toPoseAndBodyTwist2D() const = 0;
  virtual PoseAndTwist3D toPoseAndBodyTwist3D() const = 0;

protected :

  Duration duration_;
};


}

#endif
