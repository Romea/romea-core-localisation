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

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__TRAITS_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__TRAITS_HPP_

// romea
#include "romea_core_filtering/filter/kalman/filter.hpp"
#include "romea_core_filtering/filter/particle/filter.hpp"
#include "romea_core_localisation/robot_to_robot/kalman/predictor.hpp"
#include "romea_core_localisation/robot_to_robot/kalman/updater_leader_pose.hpp"
#include "romea_core_localisation/robot_to_robot/kalman/updater_range.hpp"
#include "romea_core_localisation/robot_to_robot/particle/predictor.hpp"
#include "romea_core_localisation/robot_to_robot/particle/updater_leader_pose.hpp"
#include "romea_core_localisation/robot_to_robot/particle/updater_range.hpp"
#include "romea_core_localisation/robot_to_robot/results.hpp"
#include "romea_core_localisation/robot_to_robot/updater_leader_twist.hpp"
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
struct R2RTraits
{
};

template<>
struct R2RTraits<KALMAN>
{
  using Filter = KalmanFilter<R2RKFMetaState, FSMState, Duration>;
  using MetaState = R2RKFMetaState;
  using Predictor = R2RKFPredictor;
  using ObservationAgeLimits = typename Predictor::ObservationAgeLimits;
  using UpdaterTwist = romea::core::localisation::UpdaterTwist<R2RKFMetaState>;
  using UpdaterLeaderTwist = R2RUpdaterLeaderTwist<R2RKFMetaState>;
  using UpdaterLinearSpeed = romea::core::localisation::UpdaterLinearSpeed<R2RKFMetaState>;
  using UpdaterLinearSpeeds = romea::core::localisation::UpdaterLinearSpeeds<R2RKFMetaState>;
  using UpdaterAngularSpeed = romea::core::localisation::UpdaterAngularSpeed<R2RKFMetaState>;
  using UpdaterPose = R2RKFUpdaterLeaderPose;
  using UpdaterRange = R2RKFUpdaterRange;
  using Results = R2RResults;
  using MetaStateToResults = R2RKFMetaStateToResults;
};

template<>
struct R2RTraits<PARTICLE>
{
  using Filter = ParticleFilter<R2RPFMetaState, FSMState, Duration>;
  using MetaState = R2RPFMetaState;
  using Predictor = R2RPFPredictor;
  using ObservationAgeLimits = typename Predictor::ObservationAgeLimits;
  using UpdaterTwist = romea::core::localisation::UpdaterTwist<R2RPFMetaState>;
  using UpdaterLeaderTwist = R2RUpdaterLeaderTwist<R2RPFMetaState>;
  using UpdaterLinearSpeed = romea::core::localisation::UpdaterLinearSpeed<R2RPFMetaState>;
  using UpdaterLinearSpeeds = romea::core::localisation::UpdaterLinearSpeeds<R2RPFMetaState>;
  using UpdaterAngularSpeed = romea::core::localisation::UpdaterAngularSpeed<R2RPFMetaState>;
  using UpdaterPose = R2RPFUpdaterLeaderPose;
  using UpdaterRange = R2RPFUpdaterRange;
  using Results = R2RResults;
  using MetaStateToResults = R2RPFMetaStateToResults;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__TRAITS_HPP_
