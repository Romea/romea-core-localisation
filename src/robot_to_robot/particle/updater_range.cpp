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


// std
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
  const std::string & updaterName,
  const double & minimalRate,
  const TriggerMode & triggerMode,
  const size_t & numberOfParticles,
  const double & maximalMahalanobisDistance,
  const std::string & logFilename)
: UpdaterExteroceptive(updaterName, minimalRate, triggerMode, logFilename),
  PFGaussianUpdaterCore(numberOfParticles, maximalMahalanobisDistance),
  cosCourses_(RowMajorVector::Zero(numberOfParticles_)),
  sinCourses_(RowMajorVector::Zero(numberOfParticles_))
{
}

//-----------------------------------------------------------------------------
void R2RPFUpdaterRange::update(
  const Duration & duration,
  const Observation & currentObservation,
  FSMState & currentFSMState,
  MetaState & currentMetaState)
{
  rate_diagnostic_.evaluate(duration);

  if (currentFSMState == FSMState::RUNNING) {
    try {
      update_(
        duration,
        currentObservation,
        currentMetaState.state,
        currentMetaState.addon);
    } catch (...) {
      std::cout << " FSM : FILTER DEGENERESCENCE, RESET AND GO TO INIT MODE" << std::endl;
      currentMetaState.state.reset();
      currentMetaState.addon.reset();
      currentFSMState = FSMState::INIT;
    }
  }
}

//--------------------------------------------------------------------------
void R2RPFUpdaterRange::update_(
  const Duration & duration,
  const Observation & currentObservation,
  State & currentState,
  AddOn & currentAddOn)
{
  // get position of the follower tag
  double xf = currentObservation.initiator_position.x();
  double yf = currentObservation.initiator_position.y();
  double zf = currentObservation.initiator_position.z();

  // get the position of leader tag
  double xl = currentObservation.responder_position.x();
  double yl = currentObservation.responder_position.y();
  double zl = currentObservation.responder_position.z();

  // Compute importance weights
  const auto & courses = currentState.particles.row(2);
  const auto & x = currentState.particles.row(0);
  const auto & y = currentState.particles.row(1);

  cosCourses_ = courses.array().cos();
  sinCourses_ = courses.array().sin();

  aprioriObservations_ = ((x + (cosCourses_ * xl - sinCourses_ * yl) - xf).square() +
    (y + (sinCourses_ * xl + cosCourses_ * yl) - yf).square() +
    (zl - zf) * (zl - zf)).sqrt();

  computeInnovation_(currentObservation, currentState.weights);
  bool success = updateState_(currentState, currentObservation);

  if (success) {
    currentAddOn.last_exteroceptive_update.time = duration;
    currentAddOn.last_exteroceptive_update.travelled_distance = currentAddOn.travelled_distance;
  }

  // log
  if (log_file_.is_open()) {
    log_file_ << duration.count() << " ";
    log_file_ << success << " ";
    log_file_ << currentObservation.Y() << " ";
    log_file_ << currentObservation.R() << " ";
    log_file_ << aprioriObservation_.Y() << " ";
    log_file_ << aprioriObservation_.R() << " ";
    log_file_ << this->mahalanobisDistance_ << std::endl;
  }
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
