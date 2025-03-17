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
#include <romea_core_common/math/EulerAngles.hpp>

// std
#include <iostream>
#include <string>

// local
#include "romea_core_localisation/robot_to_world/kalman/updater_course.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//--------------------------------------------------------------------------
R2WKFUpdaterCourse::R2WKFUpdaterCourse(
  const std::string & updaterName,
  const double & minimalRate,
  const TriggerMode & triggerMode,
  const double & maximalMahalanobisDistance,
  const std::string & logFilename)
: UpdaterExteroceptive(updaterName, minimalRate, triggerMode, logFilename),
  KFUpdaterCore(maximalMahalanobisDistance)
{
  H_(0, MetaState::ORIENTATION_Z) = 1;
  set_log_file_header_(
    {"stamp",
      "course",
      "cov_course",
      "theta",
      "cov_theta"
    });
}

//--------------------------------------------------------------------------
void R2WKFUpdaterCourse::update(
  const Duration & duration,
  const Observation & currentObservation,
  FSMState & currentFSMState,
  MetaState & currentMetaState)
{
  assert(currentObservation.R() > 0);

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
        try {
          update_(
            duration,
            currentObservation,
            currentMetaState.state,
            currentMetaState.addon);
        } catch (...) {
          std::cout << " FSM : COURSE UPDATE HAS FAILED, RESET AND GO TO INIT MODE" << std::endl;
          currentFSMState = FSMState::INIT;
          currentMetaState.state.reset();
          currentMetaState.addon.reset();
        }
      }
      break;
    default:
      break;
  }
}
//--------------------------------------------------------------------------
void R2WKFUpdaterCourse::update_(
  const Duration & duration,
  const Observation & currentObservation,
  State & currentState,
  AddOn & currentAddon)
{
  Inn_ = betweenMinusPiAndPi(currentObservation.Y() - currentState.X(MetaState::ORIENTATION_Z));

  QInn_ = currentObservation.R() + currentState.P(
    MetaState::ORIENTATION_Z,
    MetaState::ORIENTATION_Z);

  // log
  if (log_file_.is_open()) {
    log_file_ << duration.count() << ",";
    log_file_ << currentObservation.Y() << ",";
    log_file_ << currentObservation.R() << ",";
    log_file_ << currentState.X(2) << ",";
    log_file_ << currentState.P(2, 2) << ",/n";
  }

  if (!updateState_(currentState)) {
    // TODO(jean) renvoyer un throw
  }

  currentAddon.last_exteroceptive_update.time = duration;
  currentAddon.last_exteroceptive_update.travelled_distance = currentAddon.travelled_distance;
}

//--------------------------------------------------------------------------
void R2WKFUpdaterCourse::set_(
  const Duration & /*duration*/,
  const Observation & currentObservation,
  State & currentState,
  AddOn & /*currentAddon*/)
{
  currentState.X(MetaState::ORIENTATION_Z) = currentObservation.Y();

  currentState.P(
    MetaState::ORIENTATION_Z,
    MetaState::ORIENTATION_Z) = currentObservation.R();


//  currentAddon.lastExteroceptiveUpdate.time=duration;
//  currentAddon.lastExteroceptiveUpdate.travelledDistance=currentAddon.travelledDistance;
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
