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


// romea
#include <romea_core_common/math/NormalRandomMatrixGenerator.hpp>
#include <romea_core_common/math/EulerAngles.hpp>

// std
#include <random>
#include <string>

// local
#include "romea_core_localisation/robot_to_world/particle/updater_course.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//--------------------------------------------------------------------------
R2WPFUpdaterCourse::R2WPFUpdaterCourse(
  const std::string & updaterName,
  const double & minimalRate,
  const TriggerMode & triggerMode,
  const size_t & numberOfParticles,
  const double & /*maximalMahalanobisDistance*/,
  const std::string & logFilename)
: UpdaterExteroceptive(updaterName, minimalRate, triggerMode, logFilename),
  PFUpdaterCore(numberOfParticles)
{
}

//--------------------------------------------------------------------------
void R2WPFUpdaterCourse::update(
  const Duration & duration,
  const Observation & currentObservation,
  FSMState & currentFSMState,
  MetaState & currentMetaState)
{
  rate_diagnostic_.evaluate(duration);

  switch (currentFSMState) {
    case FSMState::INIT:
      set_(
        duration,
        currentObservation,
        currentMetaState.state,
        currentMetaState.addon);
      break;
    case FSMState::RUNNING:
      if (trigger_mode_ == TriggerMode::ALWAYS) {
        update_(
          duration,
          currentObservation,
          currentMetaState.state,
          currentMetaState.addon);
      }
      break;
    default:
      break;
  }
}

//--------------------------------------------------------------------------
void R2WPFUpdaterCourse::update_(
  const Duration & duration,
  const Observation & currentObservation,
  State & currentState,
  AddOn & currentAddon)
{
  double course = currentObservation.Y();
  double vonMisesConcentration = 0.5 / (1 - std::exp(-currentObservation.R() / 2));

  const auto & courses = currentState.particles.row(MetaState::ANGULAR_SPEED_Z_BODY);
  currentState.weights *= ((courses - course).cos() * vonMisesConcentration).exp();

  currentAddon.last_exteroceptive_update.time = duration;
  currentAddon.last_exteroceptive_update.travelled_distance = currentAddon.travelled_distance;
}

//--------------------------------------------------------------------------
void R2WPFUpdaterCourse::set_(
  const Duration & duration,
  const Observation & currentObservation,
  State & currentState,
  AddOn & currentAddon)
{
  auto particleCourses = currentState.particles.row(MetaState::ORIENTATION_Z);

  NormalRandomArrayGenerator<double> randomGenerator;
  randomGenerator.init(currentObservation.Y(), currentObservation.R());
  randomGenerator.fill(particleCourses);

  currentAddon.last_exteroceptive_update.time = duration;
  currentAddon.last_exteroceptive_update.travelled_distance = currentAddon.travelled_distance;
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
