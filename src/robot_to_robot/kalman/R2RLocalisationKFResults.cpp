// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "romea_core_localisation/robot_to_robot/kalman/R2RLocalisationKFResults.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
R2RLocalisationKFResults::R2RLocalisationKFResults()
: R2RLocalisationResults()
{
}

//--------------------------------------------------------------------------
const double & R2RLocalisationKFResults::getLeaderX() const
{
  return state.X(LEADER_POSITION_X);
}

//--------------------------------------------------------------------------
const double & R2RLocalisationKFResults::getLeaderY() const
{
  return state.X(LEADER_POSITION_Y);
}

//--------------------------------------------------------------------------
const double & R2RLocalisationKFResults::getLeaderOrientation() const
{
  return state.X(LEADER_ORIENTATION_Z);
}

//--------------------------------------------------------------------------
const double & R2RLocalisationKFResults::getLinearSpeed() const
{
  return input.U(LINEAR_SPEED_X_BODY);
}

//--------------------------------------------------------------------------
const double & R2RLocalisationKFResults::getLateralSpeed() const
{
  return input.U(LINEAR_SPEED_Y_BODY);
}

//--------------------------------------------------------------------------
const double & R2RLocalisationKFResults::getAngularSpeed() const
{
  return input.U(ANGULAR_SPEED_Z_BODY);
}

//--------------------------------------------------------------------------
Eigen::Vector3d R2RLocalisationKFResults::getTwist() const
{
  return input.U().segment<3>(LINEAR_SPEED_X_BODY);
}

//--------------------------------------------------------------------------
Eigen::Matrix3d R2RLocalisationKFResults::getTwistCovariance()const
{
  return input.QU().block<3, 3>(LINEAR_SPEED_X_BODY, LINEAR_SPEED_X_BODY);
}

//--------------------------------------------------------------------------
const double & R2RLocalisationKFResults::getLeaderLinearSpeed() const
{
  return input.U(LEADER_LINEAR_SPEED_X_BODY);
}

//--------------------------------------------------------------------------
const double & R2RLocalisationKFResults::getLeaderLateralSpeed() const
{
  return input.U(LEADER_LINEAR_SPEED_Y_BODY);
}

//--------------------------------------------------------------------------
const double & R2RLocalisationKFResults::getLeaderAngularSpeed() const
{
  return input.U(LEADER_ANGULAR_SPEED_Z_BODY);
}

//--------------------------------------------------------------------------
Eigen::Vector3d R2RLocalisationKFResults::getLeaderTwist() const
{
  return input.U().segment<3>(LEADER_LINEAR_SPEED_X_BODY);
}

//--------------------------------------------------------------------------
Eigen::Matrix3d R2RLocalisationKFResults::getLeaderTwistCovariance()const
{
  return input.QU().block<3, 3>(LEADER_LINEAR_SPEED_X_BODY, LEADER_LINEAR_SPEED_X_BODY);
}

//--------------------------------------------------------------------------
Eigen::Vector3d R2RLocalisationKFResults::getLeaderPose() const
{
  return state.X();
}

//--------------------------------------------------------------------------
Eigen::Matrix3d R2RLocalisationKFResults::getLeaderPoseCovariance() const
{
  return state.P();
}

//-----------------------------------------------------------------------------
Pose2D R2RLocalisationKFResults::toLeaderPose2D() const
{
  Pose2D pose2d;
  pose2d.position.x() = getLeaderX();
  pose2d.position.y() = getLeaderY();
  pose2d.yaw = getLeaderOrientation();
  pose2d.covariance = getLeaderPoseCovariance();
  return pose2d;
}

//-----------------------------------------------------------------------------
PoseAndTwist2D R2RLocalisationKFResults::toLeaderPoseAndBodyTwist2D() const
{
  PoseAndTwist2D poseAndTwist2D;
  poseAndTwist2D.pose.position.x() = getLeaderX();
  poseAndTwist2D.pose.position.y() = getLeaderY();
  poseAndTwist2D.pose.yaw = getLeaderOrientation();
  poseAndTwist2D.pose.covariance = getLeaderPoseCovariance();
  poseAndTwist2D.twist.linearSpeeds.x() = getLeaderLinearSpeed();
  poseAndTwist2D.twist.linearSpeeds.y() = getLeaderLateralSpeed();
  poseAndTwist2D.twist.angularSpeed = getLeaderAngularSpeed();
  poseAndTwist2D.twist.covariance = getLeaderTwistCovariance();
  return poseAndTwist2D;
}

}  // namespace romea
