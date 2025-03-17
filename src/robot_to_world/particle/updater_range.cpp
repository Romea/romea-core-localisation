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
  const std::string & updaterName,
  const double & minimalRate,
  const TriggerMode & triggerMode,
  const size_t & numberOfParticles,
  const double & maximalMahalanobisDistance,
  const std::string & logFilename)
: UpdaterExteroceptive(updaterName, minimalRate, triggerMode, logFilename),
  PFGaussianUpdaterCore(numberOfParticles, maximalMahalanobisDistance),
  cosCourses_(RowMajorVector::Zero(numberOfParticles_)),
  sinCourses_(RowMajorVector::Zero(numberOfParticles_)),
  levelArmCompensation_()
{
}

//-----------------------------------------------------------------------------
void R2WPFUpdaterRange::update(
  const Duration & duration,
  const Observation & currentObservation,
  FSMState & currentFSMState,
  MetaState & currentMetaState)
{
  if (currentFSMState == FSMState::RUNNING) {
    if (trigger_mode_ == TriggerMode::ALWAYS) {
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
}

//--------------------------------------------------------------------------
void R2WPFUpdaterRange::update_(
  const Duration & duration,
  Observation currentObservation,
  State & currentState,
  AddOn & currentAddon)
{
  // compute antenna attitude compensation
  levelArmCompensation_.compute(
    currentAddon.roll,
    currentAddon.pitch,
    currentAddon.roll_pitch_variance,
    0,
    0,
    currentObservation.initiator_position);


  const Eigen::Vector3d & tagAntennaPosition = levelArmCompensation_.getPosition();

  // compute a priori observations
  const double & eax = tagAntennaPosition.x();
  const double & eay = tagAntennaPosition.y();
  const double & eaz = tagAntennaPosition.z() + currentObservation.terrain_elevation;

  const double & iax = currentObservation.responder_position.x();
  const double & iay = currentObservation.responder_position.y();
  const double & iaz = currentObservation.responder_position.z();

  const auto & courses = currentState.particles.row(MetaState::ORIENTATION_Z);
  const auto & x = currentState.particles.row(MetaState::POSITION_X);
  const auto & y = currentState.particles.row(MetaState::POSITION_Y);

  cosCourses_ = courses.array().cos();
  sinCourses_ = courses.array().sin();

  aprioriObservations_ = ((x + (cosCourses_ * eax - sinCourses_ * eay) - iax).square() +
    (y + (sinCourses_ * eax + cosCourses_ * eay) - iay).square() +
    (eaz - iaz) * (eaz - iaz)).sqrt();

  // update weights and resample
  currentObservation.R() += levelArmCompensation_.getPositionCovariance().trace();

  if (updateState_(currentState, currentObservation)) {
    currentAddon.last_exteroceptive_update.time = duration;
    currentAddon.last_exteroceptive_update.travelled_distance = currentAddon.travelled_distance;
  }
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
