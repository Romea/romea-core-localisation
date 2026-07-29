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
#include <limits>
#include <string>

// local
#include "romea_core_localisation/robot_to_robot/particle/updater_range.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//--------------------------------------------------------------------------
R2RPFUpdaterRange::R2RPFUpdaterRange(
  const std::string & updater_name,
  const double & minimal_rate,
  const trigger_mode & trigger_mode,
  const size_t & number_of_particles,
  const double & maximal_mahalanobis_distance)
: UpdaterExteroceptive(updater_name, minimal_rate, trigger_mode),
  PFGaussianUpdaterBase(number_of_particles, maximal_mahalanobis_distance),
  cos_courses_(RowMajorVector::Zero(number_of_particles_)),
  sin_courses_(RowMajorVector::Zero(number_of_particles_))
{
}

//-----------------------------------------------------------------------------
void R2RPFUpdaterRange::update(
  const Duration & duration,
  const Observation & current_observation,
  FSMState & current_fsm_State,
  MetaState & current_meta_state)
{
  rate_diagnostic_.evaluate(duration);

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
          "RANGE UPDATE HAS FAILED, RESET AND GO TO INIT MODE");
      }
    }
  }
}

//--------------------------------------------------------------------------
void R2RPFUpdaterRange::update_(
  const Duration & duration,
  const Observation & current_observation,
  State & current_state,
  AddOn & current_add_on)
{
  // get position of the follower tag
  double xf = current_observation.initiator_position.x();
  double yf = current_observation.initiator_position.y();
  double zf = current_observation.initiator_position.z();

  // get the position of leader tag
  double xl = current_observation.responder_position.x();
  double yl = current_observation.responder_position.y();
  double zl = current_observation.responder_position.z();

  // Compute importance weights
  const auto & courses = current_state.particles.row(2);
  const auto & x = current_state.particles.row(0);
  const auto & y = current_state.particles.row(1);

  cos_courses_ = courses.array().cos();
  sin_courses_ = courses.array().sin();

  apriori_observations_ =
    ((x + (cos_courses_ * xl - sin_courses_ * yl) - xf).square() +
     (y + (sin_courses_ * xl + cos_courses_ * yl) - yf).square() + (zl - zf) * (zl - zf))
      .sqrt();

  compute_innovation_(current_observation, current_state.weights);
  bool success = update_state_(current_state, current_observation);

  if (success) {
    current_add_on.dead_reckoning_tracking.start_time = duration;
    current_add_on.dead_reckoning_tracking.start_travelled_distance = current_add_on.travelled_distance;
  }

  // log
  if (logger_) {
    logger_->addEntry("stamp", durationToSecond(duration));
    logger_->addEntry("success", success);
    logger_->addEntry("range", current_observation.Y());
    logger_->addEntry("cov_range", current_observation.R());
    logger_->addEntry("apriori_range", apriori_observation_.Y());
    logger_->addEntry("cov_apriori_range", apriori_observation_.R());
    logger_->addEntry("mahalanobis_distance", this->mahalanobis_distance_);
    logger_->addEntry(
      "effective_sample_size",
      success ? this->resampling_.get_number_of_effective_samples() :
      std::numeric_limits<double>::quiet_NaN());
    logger_->addEntry("resampled", success ? this->resampling_.has_resampled() : false);
    logger_->writeRow();
  }
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
