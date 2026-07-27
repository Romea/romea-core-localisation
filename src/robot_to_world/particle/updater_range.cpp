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

// std
#include <iostream>
#include <string>

// romea
#include "romea_core_localisation/robot_to_world/particle/updater_range.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//--------------------------------------------------------------------------
R2WPFUpdaterRange::R2WPFUpdaterRange(
  const std::string & updater_name,
  const double & minimal_rate,
  const trigger_mode & trigger_mode,
  const size_t & number_of_particles,
  const double & maximal_mahalanobis_distance)
: UpdaterExteroceptive(updater_name, minimal_rate, trigger_mode),
  PFGaussianUpdaterBase(number_of_particles, maximal_mahalanobis_distance),
  cos_courses_(RowMajorVector::Zero(number_of_particles_)),
  sin_courses_(RowMajorVector::Zero(number_of_particles_)),
  lever_arm_compensation_()
{
}

//-----------------------------------------------------------------------------
void R2WPFUpdaterRange::update(
  const Duration & duration,
  const Observation & current_observation,
  FSMState & current_fsm_State,
  MetaState & current_meta_state)
{
  if (current_fsm_State == FSMState::RUNNING) {
    if (trigger_mode_ == trigger_mode::ALWAYS) {
      try {
        update_(duration, current_observation, current_meta_state.state, current_meta_state.addon);
      } catch (...) {
        const auto previous_fsm_state = current_fsm_State;
        current_meta_state.state.reset();
        current_meta_state.addon.reset();
        current_fsm_State = FSMState::INIT;
        notify_fsm_event_(
          previous_fsm_state,
          current_fsm_State,
          "FILTER DEGENERESCENCE, RESET AND GO TO INIT MODE");
      }
    }
  }
}

//--------------------------------------------------------------------------
void R2WPFUpdaterRange::update_(
  const Duration & duration,
  Observation current_observation,
  State & current_state,
  AddOn & current_add_on)
{
  // compute antenna attitude compensation
  lever_arm_compensation_.compute(
    current_add_on.roll,
    current_add_on.pitch,
    current_add_on.roll_pitch_variance,
    0,
    0,
    current_observation.initiator_position);

  const Eigen::Vector3d & tagAntennaPosition = lever_arm_compensation_.getPosition();

  // compute a priori observations
  const double & eax = tagAntennaPosition.x();
  const double & eay = tagAntennaPosition.y();
  const double & eaz = tagAntennaPosition.z() + current_observation.terrain_elevation;

  const double & iax = current_observation.responder_position.x();
  const double & iay = current_observation.responder_position.y();
  const double & iaz = current_observation.responder_position.z();

  const auto & courses = current_state.particles.row(MetaState::ORIENTATION_Z);
  const auto & x = current_state.particles.row(MetaState::POSITION_X);
  const auto & y = current_state.particles.row(MetaState::POSITION_Y);

  cos_courses_ = courses.array().cos();
  sin_courses_ = courses.array().sin();

  apriori_observations_ =
    ((x + (cos_courses_ * eax - sin_courses_ * eay) - iax).square() +
     (y + (sin_courses_ * eax + cos_courses_ * eay) - iay).square() + (eaz - iaz) * (eaz - iaz))
      .sqrt();

  // update weights and resample
  current_observation.R() += lever_arm_compensation_.getPositionCovariance().trace();

  if (update_state_(current_state, current_observation)) {
    current_add_on.dead_reckoning_tracking.start_time = duration;
    current_add_on.dead_reckoning_tracking.start_travelled_distance = current_add_on.travelled_distance;
  }
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
