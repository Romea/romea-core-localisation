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


#include <romea_core_common/math/Matrix.hpp>
#include "romea_core_localisation/robot_to_world/kalman/meta_state.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//--------------------------------------------------------------------------
R2WKFMetaState::R2WKFMetaState()
: R2WMetaStateBase(),
  state()
{
}

//--------------------------------------------------------------------------
void apply_lever_arm_compensation(
  R2WKFMetaState::State & currentState,
  R2WKFMetaState::AddOn & currentAddOn,
  LeverArmCompensation & levelArmCompensation,
  const Eigen::Vector3d & levelArm)
{
  levelArmCompensation.compute(
    currentAddOn.roll,
    currentAddOn.pitch,
    currentAddOn.roll_pitch_variance,
    currentState.X(R2WKFMetaState::ORIENTATION_Z),
    0,
    levelArm);

  currentState.X().segment<2>(R2WKFMetaState::POSITION_X) -=
    levelArmCompensation.getPosition().segment<2>(R2WKFMetaState::POSITION_X);

  currentState.P().block<2, 2>(R2WKFMetaState::POSITION_X, R2WKFMetaState::POSITION_X) +=
    levelArmCompensation.getPositionCovariance().block<2, 2>(
    R2WKFMetaState::POSITION_X, R2WKFMetaState::POSITION_X);

  assert(isPositiveSemiDefiniteMatrix(currentState.P()));
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
