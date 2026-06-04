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

#include "romea_core_localisation/robot_to_robot/kalman/results.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
R2RKFResults::R2RKFResults() : R2RResultsBase()
{
}

//--------------------------------------------------------------------------
const double & R2RKFResults::get_leader_x() const
{
  return state.X(LEADER_POSITION_X);
}

//--------------------------------------------------------------------------
const double & R2RKFResults::get_leader_y() const
{
  return state.X(LEADER_POSITION_Y);
}

//--------------------------------------------------------------------------
const double & R2RKFResults::get_leader_orientation() const
{
  return state.X(LEADER_ORIENTATION_Z);
}

//--------------------------------------------------------------------------
const double & R2RKFResults::get_linear_speed() const
{
  return input.U(LINEAR_SPEED_X_BODY);
}

//--------------------------------------------------------------------------
const double & R2RKFResults::get_lateral_speed() const
{
  return input.U(LINEAR_SPEED_Y_BODY);
}

//--------------------------------------------------------------------------
const double & R2RKFResults::get_angular_speed() const
{
  return input.U(ANGULAR_SPEED_Z_BODY);
}

//--------------------------------------------------------------------------
Eigen::Vector3d R2RKFResults::get_twist() const
{
  return input.U().segment<3>(LINEAR_SPEED_X_BODY);
}

//--------------------------------------------------------------------------
Eigen::Matrix3d R2RKFResults::get_twist_covariance() const
{
  return input.QU().block<3, 3>(LINEAR_SPEED_X_BODY, LINEAR_SPEED_X_BODY);
}

//--------------------------------------------------------------------------
const double & R2RKFResults::get_leader_linear_speed() const
{
  return input.U(LEADER_LINEAR_SPEED_X_BODY);
}

//--------------------------------------------------------------------------
const double & R2RKFResults::get_leader_lateral_speed() const
{
  return input.U(LEADER_LINEAR_SPEED_Y_BODY);
}

//--------------------------------------------------------------------------
const double & R2RKFResults::get_leader_angular_speed() const
{
  return input.U(LEADER_ANGULAR_SPEED_Z_BODY);
}

//--------------------------------------------------------------------------
Eigen::Vector3d R2RKFResults::get_leader_twist() const
{
  return input.U().segment<3>(LEADER_LINEAR_SPEED_X_BODY);
}

//--------------------------------------------------------------------------
Eigen::Matrix3d R2RKFResults::get_leader_twist_covariance() const
{
  return input.QU().block<3, 3>(LEADER_LINEAR_SPEED_X_BODY, LEADER_LINEAR_SPEED_X_BODY);
}

//--------------------------------------------------------------------------
Eigen::Vector3d R2RKFResults::get_leader_pose() const
{
  return state.X();
}

//--------------------------------------------------------------------------
Eigen::Matrix3d R2RKFResults::get_leader_pose_covariance() const
{
  return state.P();
}

//-----------------------------------------------------------------------------
Pose2D R2RKFResults::to_leader_pose2d() const
{
  Pose2D pose2d;
  pose2d.position.x() = get_leader_x();
  pose2d.position.y() = get_leader_y();
  pose2d.yaw = get_leader_orientation();
  pose2d.covariance = get_leader_pose_covariance();
  return pose2d;
}

//-----------------------------------------------------------------------------
PoseAndTwist2D R2RKFResults::to_leader_pose_and_body_twist2d() const
{
  PoseAndTwist2D poseAndTwist2D;
  poseAndTwist2D.pose.position.x() = get_leader_x();
  poseAndTwist2D.pose.position.y() = get_leader_y();
  poseAndTwist2D.pose.yaw = get_leader_orientation();
  poseAndTwist2D.pose.covariance = get_leader_pose_covariance();
  poseAndTwist2D.twist.linearSpeeds.x() = get_leader_linear_speed();
  poseAndTwist2D.twist.linearSpeeds.y() = get_leader_lateral_speed();
  poseAndTwist2D.twist.angularSpeed = get_leader_angular_speed();
  poseAndTwist2D.twist.covariance = get_leader_twist_covariance();
  return poseAndTwist2D;
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
