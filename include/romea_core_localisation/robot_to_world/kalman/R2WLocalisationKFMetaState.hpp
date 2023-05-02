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


#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__R2WLOCALISATIONKFMETASTATE_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__R2WLOCALISATIONKFMETASTATE_HPP_


// romea
#include <romea_core_filtering/GaussianState.hpp>

// std
#include <memory>

// local
#include "romea_core_localisation/robot_to_world/R2WLocalisationMetaState.hpp"
#include "romea_core_localisation/robot_to_world/R2WLevelArmCompensation.hpp"


namespace romea {

struct R2WLocalisationKFMetaState : R2WLocalisationMetaState
{
  using State = GaussianState<double, STATE_SIZE>;

  R2WLocalisationKFMetaState();

  virtual ~R2WLocalisationKFMetaState() = default;

  State state;
};

void applyLevelArmCompensation(R2WLocalisationKFMetaState::State & currentState,
                               R2WLocalisationKFMetaState::AddOn & currentAddOn,
                               LevelArmCompensation & levelArmCompensation,
                               const Eigen::Vector3d & levelArm);

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__R2WLOCALISATIONKFMETASTATE_HPP_
