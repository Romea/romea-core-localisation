#ifndef _R2RLocalisationResults_hpp
#define _R2RLocalisationResults_hpp

//romea
#include <romea_common/time/Time.hpp>
#include <romea_common/geometry/PoseAndTwist3D.hpp>

namespace romea {

template <class State>
class R2RLocalisationResults : public State
{

public :

  template <typename... Args>
  R2RLocalisationResults(Args... args):
    State(std::forward<Args>(args)...),
    duration_(Duration::zero())
  {

  }

  virtual ~R2RLocalisationResults()=default;

  virtual void setDuration(const Duration &duration)
  {
    duration_=duration;
  }

  virtual const double & getLeaderX() const=0;
  virtual const double & getLeaderY() const=0;
  virtual const double & getLeaderOrientation() const=0;

  virtual Eigen::Vector3d getLeaderPose() const=0;
  virtual Eigen::Matrix3d getLeaderPoseCovariance() const=0;

  virtual const double & getLinearSpeed() const=0;
  virtual const double & getLateralSpeed() const=0;
  virtual const double & getAngularSpeed() const=0;

  virtual Eigen::Vector3d getTwist() const=0;
  virtual Eigen::Matrix3d getTwistCovariance() const=0;

  virtual const double & getLeaderLinearSpeed() const=0;
  virtual const double & getLeaderLateralSpeed() const=0;
  virtual const double & getLeaderAngularSpeed() const=0;

  virtual Eigen::Vector3d getLeaderTwist() const=0;
  virtual Eigen::Matrix3d getLeaderTwistCovariance() const=0;

  virtual Pose2D toLeaderPose2D() const =0;
  virtual PoseAndTwist2D toLeaderPoseAndBodyTwist2D() const = 0;

protected :

  Duration duration_;
};

}

#endif
