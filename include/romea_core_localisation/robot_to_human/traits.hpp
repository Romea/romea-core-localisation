// Copyright 2026 INRAE, French National Research Institute for Agriculture,
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

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__TRAITS_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__TRAITS_HPP_

// romea
#include "romea_core_filtering/filter/kalman/filter.hpp"
#include "romea_core_localisation/robot_to_human/kalman/predictor.hpp"
#include "romea_core_localisation/robot_to_human/kalman/updater_leader_position.hpp"
#include "romea_core_localisation/robot_to_human/kalman/updater_range.hpp"
#include "romea_core_localisation/robot_to_human/results.hpp"
#include "romea_core_localisation/updater_angular_speed.hpp"
#include "romea_core_localisation/updater_linear_speed.hpp"
#include "romea_core_localisation/updater_linear_speeds.hpp"
#include "romea_core_localisation/updater_twist.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

template<FilterType type>
struct R2HTraits
{
};

template<>
struct R2HTraits<KALMAN>
{
  using Filter = KalmanFilter<R2HKFMetaState, FSMState, Duration>;
  using MetaState = R2HKFMetaState;
  using Predictor = R2HKFPredictor;
  using ObservationAgeLimits = typename Predictor::ObservationAgeLimits;
  using UpdaterTwist = romea::core::localisation::UpdaterTwist<R2HKFMetaState>;
  using UpdaterLinearSpeed = romea::core::localisation::UpdaterLinearSpeed<R2HKFMetaState>;
  using UpdaterLinearSpeeds = romea::core::localisation::UpdaterLinearSpeeds<R2HKFMetaState>;
  using UpdaterAngularSpeed = romea::core::localisation::UpdaterAngularSpeed<R2HKFMetaState>;
  using UpdaterPosition = R2HKFUpdaterLeaderPosition;
  using UpdaterRange = R2HKFUpdaterRange;
  using Results = R2HResults;
  using MetaStateToResults = R2HKFMetaStateToResults;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__TRAITS_HPP_
