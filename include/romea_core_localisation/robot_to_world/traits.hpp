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


#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__TRAITS_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__TRAITS_HPP_

// romea
#include <romea_core_filtering/kalman/KalmanFilter.hpp>
#include <romea_core_filtering/particle/ParticleFilter.hpp>
#include "romea_core_localisation/updater_twist.hpp"
#include "romea_core_localisation/updater_linear_speed.hpp"
#include "romea_core_localisation/updater_linear_speeds.hpp"
#include "romea_core_localisation/updater_angular_speed.hpp"
#include "romea_core_localisation/robot_to_world/updater_attitude.hpp"
#include "romea_core_localisation/robot_to_world/kalman/results.hpp"
#include "romea_core_localisation/robot_to_world/kalman/updater_course.hpp"
#include "romea_core_localisation/robot_to_world/kalman/updater_range.hpp"
#include "romea_core_localisation/robot_to_world/kalman/updater_pose.hpp"
#include "romea_core_localisation/robot_to_world/kalman/updater_position.hpp"
#include "romea_core_localisation/robot_to_world/kalman/predictor.hpp"
#include "romea_core_localisation/robot_to_world/particle/results.hpp"
#include "romea_core_localisation/robot_to_world/particle/updater_course.hpp"
#include "romea_core_localisation/robot_to_world/particle/updater_range.hpp"
#include "romea_core_localisation/robot_to_world/particle/updater_pose.hpp"
#include "romea_core_localisation/robot_to_world/particle/updater_position.hpp"
#include "romea_core_localisation/robot_to_world/particle/predictor.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

template<FilterType type>
struct R2WTraits
{
}
;

template<>
struct R2WTraits<KALMAN>
{
  using Filter = romea::core::KalmanFilter<R2WKFMetaState, FSMState, Duration>;
  using UpdaterTwist = romea::core::localisation::UpdaterTwist<R2WKFMetaState>;
  using UpdaterLinearSpeed = romea::core::localisation::UpdaterLinearSpeed<R2WKFMetaState>;
  using UpdaterLinearSpeeds = romea::core::localisation::UpdaterLinearSpeeds<R2WKFMetaState>;
  using UpdaterAngularSpeed = romea::core::localisation::UpdaterAngularSpeed<R2WKFMetaState>;
  using UpdaterAttitude = R2WUpdaterAttitude<R2WKFMetaState>;
  using UpdaterCourse = R2WKFUpdaterCourse;
  using UpdaterPose = R2WKFUpdaterPose;
  using UpdaterPosition = R2WKFUpdaterPosition;
  using UpdaterRange = R2WKFUpdaterRange;
  using Predictor = R2WKFPredictor;
  using Results = R2WKFResults;
};

template<>
struct R2WTraits<PARTICLE>
{
  using Filter = romea::core::ParticleFilter<R2WPFMetaState, FSMState, Duration>;
  using UpdaterTwist = romea::core::localisation::UpdaterTwist<R2WPFMetaState>;
  using UpdaterLinearSpeed = romea::core::localisation::UpdaterLinearSpeed<R2WPFMetaState>;
  using UpdaterLinearSpeeds = romea::core::localisation::UpdaterLinearSpeeds<R2WPFMetaState>;
  using UpdaterAngularSpeed = romea::core::localisation::UpdaterAngularSpeed<R2WPFMetaState>;
  using UpdaterAttitude = R2WUpdaterAttitude<R2WPFMetaState>;
  using UpdaterCourse = R2WPFUpdaterCourse;
  using UpdaterPose = R2WPFUpdaterPose;
  using UpdaterPosition = R2WPFUpdaterPosition;
  using UpdaterRange = R2WPFUpdaterRange;
  using Predictor = R2WPFPredictor;
  using Results = R2WPFResults;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__TRAITS_HPP_
