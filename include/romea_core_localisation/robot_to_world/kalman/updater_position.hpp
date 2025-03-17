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


#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__UPDATER_POSITION_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__UPDATER_POSITION_HPP_

// std
#include <string>

// romea
#include "romea_core_common/time/Time.hpp"
#include "romea_core_common/math/Matrix.hpp"
#include "romea_core_filtering/kalman/KalmanFilterUpdaterCore.hpp"
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/observation_position.hpp"
#include "romea_core_localisation/updater_exteroceptive.hpp"
#include "romea_core_localisation/robot_to_world/level_arm_compensation.hpp"
#include "romea_core_localisation/robot_to_world/kalman/meta_state.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

class R2WKFUpdaterPosition: public UpdaterExteroceptive, public KFUpdaterCore<double, 3, 2>
{
public:
  using Observation = ObservationPosition;
  using MetaState = R2WKFMetaState;
  using State = R2WKFMetaState::State;
  using Input = R2WKFMetaState::Input;
  using AddOn = R2WKFMetaState::AddOn;

public:
  R2WKFUpdaterPosition(
    const std::string & updaterName,
    const double & minimalRate,
    const TriggerMode & triggerMode,
    const double & maximalMahalanobisDistance,
    const std::string & logFilename);

  void update(
    const Duration & duration,
    const Observation & currentObservation,
    FSMState & currentFSMState,
    MetaState & currentMetaState);

  void update_(
    const Duration & duration,
    const Observation & currentObservation,
    State & currentState,
    AddOn & currentAddon);

  bool set_(
    const Duration & duration,
    const Observation & currentObservation,
    const Input & currentInput,
    State & currentState,
    AddOn & currentAddon);

private:
  LevelArmCompensation levelArmCompensation_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__UPDATER_POSITION_HPP_
