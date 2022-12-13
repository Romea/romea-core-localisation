#ifndef ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_KALMAN_R2WLOCALISATIONKFRESULTS_HPP_
#define ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_KALMAN_R2WLOCALISATIONKFRESULTS_HPP_

#include "romea_core_localisation/robot_to_world/R2WLocalisationResults.hpp"
#include "romea_core_localisation/robot_to_world/kalman/R2WLocalisationKFMetaState.hpp"

namespace romea {


class R2WLocalisationKFResults : public R2WLocalisationResults<R2WLocalisationKFMetaState>
{
public :

  R2WLocalisationKFResults();

  const double & getX() const override;
  const double & getY() const override;
  const double & getYaw() const override;
  const double & getYawVariance() const override;

  Eigen::Vector3d getPose() const override;
  Eigen::Matrix3d getPoseCovariance() const override;

  const double & getLinearSpeed() const override;
  const double & getLateralSpeed() const override;
  const double & getAngularSpeed() const override;

  Eigen::Vector3d getTwist() const override;
  Eigen::Matrix3d getTwistCovariance() const override;

  Pose2D toPose2D() const override;
  Pose3D toPose3D() const override;

  PoseAndTwist2D toPoseAndBodyTwist2D() const override;
  PoseAndTwist3D toPoseAndBodyTwist3D() const override;
};

}  // namespace romea

#endif   // ROMEA_CORE_LOCALISATION_ROBOT_TO_WORLD_KALMAN_R2WLOCALISATIONKFRESULTS_HPP_
