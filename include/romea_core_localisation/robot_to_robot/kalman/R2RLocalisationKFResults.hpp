#ifndef ROMEA_CORE_LOCALISATION_ROBOT_TO_ROBOT_KALMAN_R2RLOCALISATIONKFRESULTS_HPP_
#define ROMEA_CORE_LOCALISATION_ROBOT_TO_ROBOT_KALMAN_R2RLOCALISATIONKFRESULTS_HPP_

#include "romea_core_localisation/robot_to_robot/R2RLocalisationResults.hpp"
#include "romea_core_localisation/robot_to_robot/kalman/R2RLocalisationKFMetaState.hpp"

namespace romea {

class R2RLocalisationKFResults : public R2RLocalisationResults<R2RLocalisationKFMetaState>
{
public :

  R2RLocalisationKFResults();
  virtual ~R2RLocalisationKFResults() = default;

  const double & getLeaderX() const override;
  const double & getLeaderY() const override;
  const double & getLeaderOrientation() const override;

  Eigen::Vector3d getLeaderPose() const override;
  Eigen::Matrix3d getLeaderPoseCovariance() const override;

  const double & getLinearSpeed() const override;
  const double & getLateralSpeed() const override;
  const double & getAngularSpeed() const override;

  Eigen::Vector3d getTwist() const override;
  Eigen::Matrix3d getTwistCovariance() const override;

  const double & getLeaderLinearSpeed() const override;
  const double & getLeaderLateralSpeed() const override;
  const double & getLeaderAngularSpeed() const override;

  Eigen::Vector3d getLeaderTwist() const override;
  Eigen::Matrix3d getLeaderTwistCovariance() const override;

  Pose2D toLeaderPose2D() const override;
  PoseAndTwist2D toLeaderPoseAndBodyTwist2D() const override;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_ROBOT_TO_ROBOT_KALMAN_R2RLOCALISATIONKFRESULTS_HPP_
