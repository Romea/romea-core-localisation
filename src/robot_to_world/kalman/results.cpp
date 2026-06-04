// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "romea_core_localisation/robot_to_world/kalman/results.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
R2WKFResults::R2WKFResults() : R2WResultsBase()
{
}

//-----------------------------------------------------------------------------
Pose2D R2WKFResults::to_pose2d() const
{
  Pose2D pose2d;
  pose2d.position.x() = get_x();
  pose2d.position.y() = get_y();
  pose2d.yaw = get_yaw(), pose2d.covariance = get_pose_covariance();
  return pose2d;
}

//-----------------------------------------------------------------------------
Pose3D R2WKFResults::to_pose3d() const
{
  Pose3D pose3d;
  pose3d.position.x() = get_x();
  pose3d.position.y() = get_y();
  pose3d.orientation.x() = addon.roll;
  pose3d.orientation.y() = addon.pitch;
  pose3d.orientation.z() = get_yaw();
  pose3d.covariance = toSe3Covariance(get_pose_covariance());
  pose3d.covariance(3, 3) = pose3d.covariance(4, 4) = addon.roll_pitch_variance;
  return pose3d;
}

//-----------------------------------------------------------------------------
PoseAndTwist2D R2WKFResults::to_pose_and_body_twist2d() const
{
  PoseAndTwist2D poseAndTwist2D;
  poseAndTwist2D.pose.position.x() = get_x();
  poseAndTwist2D.pose.position.y() = get_y();
  poseAndTwist2D.pose.yaw = get_yaw();
  poseAndTwist2D.pose.covariance = get_pose_covariance();
  poseAndTwist2D.twist.linearSpeeds.x() = get_linear_speed();
  poseAndTwist2D.twist.linearSpeeds.y() = get_lateral_speed();
  poseAndTwist2D.twist.angularSpeed = get_angular_speed();
  poseAndTwist2D.twist.covariance = get_twist_covariance();
  return poseAndTwist2D;
}

//-----------------------------------------------------------------------------
PoseAndTwist3D R2WKFResults::to_pose_and_body_twist3d() const
{
  PoseAndTwist3D poseAndTwist3D;
  poseAndTwist3D.pose.position.x() = get_x();
  poseAndTwist3D.pose.position.y() = get_y();
  poseAndTwist3D.pose.orientation.x() = addon.roll;
  poseAndTwist3D.pose.orientation.y() = addon.pitch;
  poseAndTwist3D.pose.orientation.z() = get_yaw();
  poseAndTwist3D.twist.linearSpeeds.x() = get_linear_speed();
  poseAndTwist3D.twist.linearSpeeds.y() = get_lateral_speed();
  poseAndTwist3D.twist.angularSpeeds.z() = get_angular_speed();
  poseAndTwist3D.pose.covariance = toSe3Covariance(get_pose_covariance());
  poseAndTwist3D.twist.covariance = toSe3Covariance(get_twist_covariance());
  poseAndTwist3D.pose.covariance(3, 3) = addon.roll_pitch_variance;
  poseAndTwist3D.pose.covariance(4, 4) = addon.roll_pitch_variance;
  return poseAndTwist3D;
}

//--------------------------------------------------------------------------
const double & R2WKFResults::get_x() const
{
  return state.X(POSITION_X);
}

//--------------------------------------------------------------------------
const double & R2WKFResults::get_y() const
{
  return state.X(POSITION_Y);
}

//--------------------------------------------------------------------------
const double & R2WKFResults::get_yaw() const
{
  return state.X(ORIENTATION_Z);
}

//--------------------------------------------------------------------------
const double & R2WKFResults::get_yaw_variance() const
{
  return state.P(ORIENTATION_Z, ORIENTATION_Z);
}

//--------------------------------------------------------------------------
const double & R2WKFResults::get_linear_speed() const
{
  return input.U(LINEAR_SPEED_X_BODY);
}

//--------------------------------------------------------------------------
const double & R2WKFResults::get_lateral_speed() const
{
  return input.U(LINEAR_SPEED_Y_BODY);
}

//--------------------------------------------------------------------------
const double & R2WKFResults::get_angular_speed() const
{
  return input.U(ANGULAR_SPEED_Z_BODY);
}

//--------------------------------------------------------------------------
Eigen::Vector3d R2WKFResults::get_twist() const
{
  return input.U();
}

//--------------------------------------------------------------------------
Eigen::Matrix3d R2WKFResults::get_twist_covariance() const
{
  return input.QU();
}

//--------------------------------------------------------------------------
Eigen::Vector3d R2WKFResults::get_pose() const
{
  return state.X();
}

//--------------------------------------------------------------------------
Eigen::Matrix3d R2WKFResults::get_pose_covariance() const
{
  return state.P();
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
