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


#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__R2WLOCALISATIONTRAITS_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__R2WLOCALISATIONTRAITS_HPP_

// romea
#include <romea_core_filtering/kalman/KalmanFilter.hpp>
#include <romea_core_filtering/particle/ParticleFilter.hpp>
#include "romea_core_localisation/LocalisationUpdaterTwist.hpp"
#include "romea_core_localisation/LocalisationUpdaterLinearSpeed.hpp"
#include "romea_core_localisation/LocalisationUpdaterLinearSpeeds.hpp"
#include "romea_core_localisation/LocalisationUpdaterAngularSpeed.hpp"
#include "romea_core_localisation/robot_to_world/R2WLocalisationUpdaterAttitude.hpp"
#include "romea_core_localisation/robot_to_world/kalman/R2WLocalisationKFResults.hpp"
#include "romea_core_localisation/robot_to_world/kalman/R2WLocalisationKFUpdaterCourse.hpp"
#include "romea_core_localisation/robot_to_world/kalman/R2WLocalisationKFUpdaterRange.hpp"
#include "romea_core_localisation/robot_to_world/kalman/R2WLocalisationKFUpdaterPose.hpp"
#include "romea_core_localisation/robot_to_world/kalman/R2WLocalisationKFUpdaterPosition.hpp"
#include "romea_core_localisation/robot_to_world/kalman/R2WLocalisationKFPredictor.hpp"
#include "romea_core_localisation/robot_to_world/particle/R2WLocalisationPFResults.hpp"
#include "romea_core_localisation/robot_to_world/particle/R2WLocalisationPFUpdaterCourse.hpp"
#include "romea_core_localisation/robot_to_world/particle/R2WLocalisationPFUpdaterRange.hpp"
#include "romea_core_localisation/robot_to_world/particle/R2WLocalisationPFUpdaterPose.hpp"
#include "romea_core_localisation/robot_to_world/particle/R2WLocalisationPFUpdaterPosition.hpp"
#include "romea_core_localisation/robot_to_world/particle/R2WLocalisationPFPredictor.hpp"

namespace romea
{

template<FilterType type>
struct R2WLocalisationTraits
{
}
;

template<>
struct R2WLocalisationTraits<KALMAN>
{
  using Filter = KalmanFilter<R2WLocalisationKFMetaState, LocalisationFSMState, Duration>;
  using UpdaterTwist = LocalisationUpdaterTwist<R2WLocalisationKFMetaState>;
  using UpdaterLinearSpeed = LocalisationUpdaterLinearSpeed<R2WLocalisationKFMetaState>;
  using UpdaterLinearSpeeds = LocalisationUpdaterLinearSpeeds<R2WLocalisationKFMetaState>;
  using UpdaterAngularSpeed = LocalisationUpdaterAngularSpeed<R2WLocalisationKFMetaState>;
  using UpdaterAttitude = R2WLocalisationUpdaterAttitude<R2WLocalisationKFMetaState>;
  using UpdaterCourse = R2WLocalisationKFUpdaterCourse;
  using UpdaterPose = R2WLocalisationKFUpdaterPose;
  using UpdaterPosition = R2WLocalisationKFUpdaterPosition;
  using UpdaterRange = R2WLocalisationKFUpdaterRange;
  using Predictor = R2WLocalisationKFPredictor;
  using Results = R2WLocalisationKFResults;
};

template<>
struct R2WLocalisationTraits<PARTICLE>
{
  using Filter = ParticleFilter<R2WLocalisationPFMetaState, LocalisationFSMState, Duration>;
  using UpdaterTwist = LocalisationUpdaterTwist<R2WLocalisationPFMetaState>;
  using UpdaterLinearSpeed = LocalisationUpdaterLinearSpeed<R2WLocalisationPFMetaState>;
  using UpdaterLinearSpeeds = LocalisationUpdaterLinearSpeeds<R2WLocalisationPFMetaState>;
  using UpdaterAngularSpeed = LocalisationUpdaterAngularSpeed<R2WLocalisationPFMetaState>;
  using UpdaterAttitude = R2WLocalisationUpdaterAttitude<R2WLocalisationPFMetaState>;
  using UpdaterCourse = R2WLocalisationPFUpdaterCourse;
  using UpdaterPose = R2WLocalisationPFUpdaterPose;
  using UpdaterPosition = R2WLocalisationPFUpdaterPosition;
  using UpdaterRange = R2WLocalisationPFUpdaterRange;
  using Predictor = R2WLocalisationPFPredictor;
  using Results = R2WLocalisationPFResults;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__R2WLOCALISATIONTRAITS_HPP_
