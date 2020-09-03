#ifndef _R2RLocalisationKFResults_hpp
#define _R2RLocalisationKFResults_hpp

#include "../R2RLocalisationResults.hpp"
#include "R2RLocalisationKFMetaState.hpp"

namespace romea {

class R2RLocalisationKFResults : public R2RLocalisationResults<R2RLocalisationKFMetaState>
{

public :

  R2RLocalisationKFResults();
  virtual ~R2RLocalisationKFResults()=default;

  virtual const double & getLeaderX() const override;
  virtual const double & getLeaderY() const override;
  virtual const double & getLeaderOrientation() const override;

  virtual Eigen::Vector3d getLeaderPose() const override;
  virtual Eigen::Matrix3d getLeaderPoseCovariance() const override;

  virtual const double & getLinearSpeed() const override;
  virtual const double & getLateralSpeed() const override;
  virtual const double & getAngularSpeed() const override;

  virtual Eigen::Vector3d getTwist() const override;
  virtual Eigen::Matrix3d getTwistCovariance() const override;

  const double & getLeaderLinearSpeed() const override;
  const double & getLeaderLateralSpeed() const override;
  const double & getLeaderAngularSpeed() const override;

  Eigen::Vector3d getLeaderTwist() const override;
  Eigen::Matrix3d getLeaderTwistCovariance() const override;

  virtual Pose2D toLeaderPose2D() const override;
  virtual PoseAndTwist2D toLeaderPoseAndBodyTwist2D() const override;

};

}

#endif
