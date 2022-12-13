#include "romea_core_localisation/robot_to_world/kalman/R2WLocalisationKFResults.hpp"

namespace romea {

//-----------------------------------------------------------------------------
R2WLocalisationKFResults::R2WLocalisationKFResults():
  R2WLocalisationResults()
{
}

//-----------------------------------------------------------------------------
Pose2D R2WLocalisationKFResults::toPose2D() const
{
  Pose2D pose2d;
  pose2d.position.x() = getX();
  pose2d.position.y() = getY();
  pose2d.yaw = getYaw(),
  pose2d.covariance = getPoseCovariance();
  return pose2d;
}

//-----------------------------------------------------------------------------
Pose3D R2WLocalisationKFResults::toPose3D() const
{
  Pose3D pose3d;
  pose3d.position.x() = getX();
  pose3d.position.y() = getY();
  pose3d.orientation.x() = addon.roll;
  pose3d.orientation.y() = addon.pitch;
  pose3d.orientation.z() = getYaw();
  pose3d.covariance = toSe3Covariance(getPoseCovariance());
  pose3d.covariance(3, 3) = pose3d.covariance(4, 4) = addon.rollPitchVariance;
  return pose3d;
}

//-----------------------------------------------------------------------------
PoseAndTwist2D R2WLocalisationKFResults::toPoseAndBodyTwist2D() const
{
  PoseAndTwist2D poseAndTwist2D;
  poseAndTwist2D.pose.position.x() = getX();
  poseAndTwist2D.pose.position.y() = getY();
  poseAndTwist2D.pose.yaw = getYaw();
  poseAndTwist2D.pose.covariance = getPoseCovariance();
  poseAndTwist2D.twist.linearSpeeds.x() = getLinearSpeed();
  poseAndTwist2D.twist.linearSpeeds.y() = getLateralSpeed();
  poseAndTwist2D.twist.angularSpeed = getAngularSpeed();
  poseAndTwist2D.twist.covariance = getTwistCovariance();
  return poseAndTwist2D;
}

//-----------------------------------------------------------------------------
PoseAndTwist3D R2WLocalisationKFResults::toPoseAndBodyTwist3D() const
{
  PoseAndTwist3D poseAndTwist3D;
  poseAndTwist3D.pose.position.x() = getX();
  poseAndTwist3D.pose.position.y() = getY();
  poseAndTwist3D.pose.orientation.x() = addon.roll;
  poseAndTwist3D.pose.orientation.y() = addon.pitch;
  poseAndTwist3D.pose.orientation.z() = getYaw();
  poseAndTwist3D.twist.linearSpeeds.x() = getLinearSpeed();
  poseAndTwist3D.twist.linearSpeeds.y() = getLateralSpeed();
  poseAndTwist3D.twist.angularSpeeds.z() = getAngularSpeed();
  poseAndTwist3D.pose.covariance = toSe3Covariance(getPoseCovariance());
  poseAndTwist3D.twist.covariance = toSe3Covariance(getTwistCovariance());
  poseAndTwist3D.pose.covariance(3, 3) = addon.rollPitchVariance;
  poseAndTwist3D.pose.covariance(4, 4) = addon.rollPitchVariance;
  return poseAndTwist3D;
}

//--------------------------------------------------------------------------
const double &R2WLocalisationKFResults::getX() const
{
  return state.X(POSITION_X);
}

//--------------------------------------------------------------------------
const double &R2WLocalisationKFResults::getY() const
{
  return state.X(POSITION_Y);
}

//--------------------------------------------------------------------------
const double &R2WLocalisationKFResults::getYaw() const
{
  return state.X(ORIENTATION_Z);
}

//--------------------------------------------------------------------------
const double & R2WLocalisationKFResults::getYawVariance() const
{
  return state.P(ORIENTATION_Z, ORIENTATION_Z);
}

//--------------------------------------------------------------------------
const double &R2WLocalisationKFResults::getLinearSpeed() const
{
  return input.U(LINEAR_SPEED_X_BODY);
}

//--------------------------------------------------------------------------
const double & R2WLocalisationKFResults::getLateralSpeed() const
{
  return input.U(LINEAR_SPEED_Y_BODY);
}

//--------------------------------------------------------------------------
const double &R2WLocalisationKFResults::getAngularSpeed() const
{
  return input.U(ANGULAR_SPEED_Z_BODY);
}

//--------------------------------------------------------------------------
Eigen::Vector3d R2WLocalisationKFResults::getTwist() const
{
  return input.U();
}

//--------------------------------------------------------------------------
Eigen::Matrix3d R2WLocalisationKFResults::getTwistCovariance()const
{
  return input.QU();
}

//--------------------------------------------------------------------------
Eigen::Vector3d R2WLocalisationKFResults::getPose() const
{
  return state.X();
}

//--------------------------------------------------------------------------
Eigen::Matrix3d R2WLocalisationKFResults::getPoseCovariance() const
{
  return state.P();
}

}  // namespace romea

