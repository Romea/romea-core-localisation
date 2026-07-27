// Copyright 2022 INRAE, French National Research Institute for Agriculture,
// Food and Environment
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

#include "romea_core_localisation/robot_to_world/kalman/meta_state.hpp"

#include <romea_core_common/math/Matrix.hpp>

namespace romea
{
namespace core
{
namespace localisation
{

//--------------------------------------------------------------------------
R2WKFMetaState::R2WKFMetaState() : R2WMetaStateBase(), state()
{
}

//--------------------------------------------------------------------------
R2WResults R2WKFMetaStateToResults::convert(const R2WKFMetaState & meta_state) const
{
  R2WResults results;

  results.robot_pose.position.x() = meta_state.state.X(R2WKFMetaState::POSITION_X);
  results.robot_pose.position.y() = meta_state.state.X(R2WKFMetaState::POSITION_Y);
  results.robot_pose.orientation.x() = meta_state.addon.roll;
  results.robot_pose.orientation.y() = meta_state.addon.pitch;
  results.robot_pose.orientation.z() = meta_state.state.X(R2WKFMetaState::ORIENTATION_Z);
  results.robot_pose.covariance = toSe3Covariance(meta_state.state.P());
  results.robot_pose.covariance(3, 3) = meta_state.addon.roll_pitch_variance;
  results.robot_pose.covariance(4, 4) = meta_state.addon.roll_pitch_variance;

  results.robot_twist.linearSpeeds.x() = meta_state.input.U(R2WKFMetaState::LINEAR_SPEED_X_BODY);
  results.robot_twist.linearSpeeds.y() = meta_state.input.U(R2WKFMetaState::LINEAR_SPEED_Y_BODY);
  results.robot_twist.angularSpeeds.z() = meta_state.input.U(R2WKFMetaState::ANGULAR_SPEED_Z_BODY);
  results.robot_twist.covariance = toSe3Covariance(meta_state.input.QU());

  return results;
}

//--------------------------------------------------------------------------
void apply_lever_arm_compensation(
  R2WKFMetaState::State & current_state,
  R2WKFMetaState::AddOn & current_add_on,
  LeverArmCompensation & lever_arm_compensation,
  const Eigen::Vector3d & lever_arm)
{
  lever_arm_compensation.compute(
    current_add_on.roll,
    current_add_on.pitch,
    current_add_on.roll_pitch_variance,
    current_state.X(R2WKFMetaState::ORIENTATION_Z),
    0,
    lever_arm);

  current_state.X().segment<2>(R2WKFMetaState::POSITION_X) -=
    lever_arm_compensation.getPosition().segment<2>(R2WKFMetaState::POSITION_X);

  current_state.P().block<2, 2>(R2WKFMetaState::POSITION_X, R2WKFMetaState::POSITION_X) +=
    lever_arm_compensation.getPositionCovariance().block<2, 2>(
      R2WKFMetaState::POSITION_X, R2WKFMetaState::POSITION_X);

  assert(isPositiveSemiDefiniteMatrix(current_state.P()));
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
