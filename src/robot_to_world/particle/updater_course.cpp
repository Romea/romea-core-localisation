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

// romea
#include <romea_core_common/math/EulerAngles.hpp>
#include <romea_core_common/math/NormalRandomMatrixGenerator.hpp>

// std
#include <random>
#include <string>

// local
#include "romea_core_localisation/robot_to_world/particle/updater_course.hpp"

namespace romea {
namespace core {
namespace localisation {

//--------------------------------------------------------------------------
R2WPFUpdaterCourse::R2WPFUpdaterCourse(
    const std::string& updater_name, const double& minimal_rate,
    const trigger_mode& trigger_mode, const size_t& number_of_particles,
    const double& /*maximal_mahalanobis_distance*/,
    const std::string& logFilename)
    : UpdaterExteroceptive(updater_name, minimal_rate, trigger_mode,
                           logFilename),
      PFUpdaterBase(number_of_particles) {}

//--------------------------------------------------------------------------
void R2WPFUpdaterCourse::update(const Duration& duration,
                                const Observation& current_observation,
                                FSMState& current_fsm_State,
                                MetaState& current_meta_state) {
  rate_diagnostic_.evaluate(duration);

  switch (current_fsm_State) {
    case FSMState::INIT:
      set_(duration, current_observation, current_meta_state.state,
           current_meta_state.addon);
      break;
    case FSMState::RUNNING:
      if (trigger_mode_ == trigger_mode::ALWAYS) {
        update_(duration, current_observation, current_meta_state.state,
                current_meta_state.addon);
      }
      break;
    default:
      break;
  }
}

//--------------------------------------------------------------------------
void R2WPFUpdaterCourse::update_(const Duration& duration,
                                 const Observation& current_observation,
                                 State& current_state, AddOn& current_add_on) {
  double course = current_observation.Y();
  double vonMisesConcentration =
      0.5 / (1 - std::exp(-current_observation.R() / 2));

  const auto& courses =
      current_state.particles.row(MetaState::ANGULAR_SPEED_Z_BODY);
  current_state.weights *=
      ((courses - course).cos() * vonMisesConcentration).exp();

  current_add_on.last_exteroceptive_update.time = duration;
  current_add_on.last_exteroceptive_update.travelled_distance =
      current_add_on.travelled_distance;
}

//--------------------------------------------------------------------------
void R2WPFUpdaterCourse::set_(const Duration& duration,
                              const Observation& current_observation,
                              State& current_state, AddOn& current_add_on) {
  auto particleCourses = current_state.particles.row(MetaState::ORIENTATION_Z);

  NormalRandomArrayGenerator<double> randomGenerator;
  randomGenerator.init(current_observation.Y(), current_observation.R());
  randomGenerator.fill(particleCourses);

  current_add_on.last_exteroceptive_update.time = duration;
  current_add_on.last_exteroceptive_update.travelled_distance =
      current_add_on.travelled_distance;
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
