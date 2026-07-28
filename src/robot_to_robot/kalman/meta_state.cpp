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

#include "romea_core_localisation/robot_to_robot/kalman/meta_state.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
R2RKFMetaState::R2RKFMetaState() : R2RMetaStateBase(), state()
{
}

//-----------------------------------------------------------------------------
R2RResults R2RKFMetaStateToResults::convert(const R2RKFMetaState & meta_state) const
{
  R2RResults results;

  results.leader_pose_and_twist.pose.position.x() =
    meta_state.state.X(R2RKFMetaState::LEADER_POSITION_X);
  results.leader_pose_and_twist.pose.position.y() =
    meta_state.state.X(R2RKFMetaState::LEADER_POSITION_Y);
  results.leader_pose_and_twist.pose.yaw = meta_state.state.X(R2RKFMetaState::LEADER_ORIENTATION_Z);
  results.leader_pose_and_twist.pose.covariance = meta_state.state.P();

  results.follower_twist.linearSpeeds.x() =
    meta_state.input.U(R2RKFMetaState::LINEAR_SPEED_X_BODY);
  results.follower_twist.linearSpeeds.y() =
    meta_state.input.U(R2RKFMetaState::LINEAR_SPEED_Y_BODY);
  results.follower_twist.angularSpeed =
    meta_state.input.U(R2RKFMetaState::ANGULAR_SPEED_Z_BODY);
  results.follower_twist.covariance = meta_state.input.QU().block<3, 3>(
    R2RKFMetaState::LINEAR_SPEED_X_BODY, R2RKFMetaState::LINEAR_SPEED_X_BODY);

  results.leader_pose_and_twist.twist.linearSpeeds.x() =
    meta_state.input.U(R2RKFMetaState::LEADER_LINEAR_SPEED_X_BODY);
  results.leader_pose_and_twist.twist.linearSpeeds.y() =
    meta_state.input.U(R2RKFMetaState::LEADER_LINEAR_SPEED_Y_BODY);
  results.leader_pose_and_twist.twist.angularSpeed =
    meta_state.input.U(R2RKFMetaState::LEADER_ANGULAR_SPEED_Z_BODY);
  results.leader_pose_and_twist.twist.covariance = meta_state.input.QU().block<3, 3>(
    R2RKFMetaState::LEADER_LINEAR_SPEED_X_BODY,
    R2RKFMetaState::LEADER_LINEAR_SPEED_X_BODY);

  return results;
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
