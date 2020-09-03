#ifndef _R2WLocalisationKFResults_hpp
#define _R2WLocalisationKFResults_hpp

#include "../R2WLocalisationResults.hpp"
#include "R2WLocalisationKFMetaState.hpp"

namespace romea {


class R2WLocalisationKFResults : public R2WLocalisationResults<R2WLocalisationKFMetaState>
{

public :

  R2WLocalisationKFResults();

  virtual const double & getX() const override;
  virtual const double & getY() const override;
  virtual const double & getYaw() const override;
  virtual const double & getYawVariance() const override;

  virtual Eigen::Vector3d getPose() const override;
  virtual Eigen::Matrix3d getPoseCovariance() const override;

  virtual const double & getLinearSpeed() const override;
  virtual const double & getLateralSpeed() const override;
  virtual const double & getAngularSpeed() const override;

  virtual Eigen::Vector3d getTwist() const override;
  virtual Eigen::Matrix3d getTwistCovariance() const override;

  virtual Pose2D toPose2D() const override;
  virtual Pose3D toPose3D() const override;

  virtual PoseAndTwist2D toPoseAndBodyTwist2D() const override;
  virtual PoseAndTwist3D toPoseAndBodyTwist3D() const override;

};


}

#endif
