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

// std
#include <iostream>
#include <string>

// local
#include "romea_core_localisation/robot_to_world/kalman/updater_course.hpp"

namespace romea {
namespace core {
namespace localisation {

//--------------------------------------------------------------------------
R2WKFUpdaterCourse::R2WKFUpdaterCourse(
    const std::string& updater_name, const double& minimal_rate,
    const trigger_mode& trigger_mode,
    const double& maximal_mahalanobis_distance, const std::string& logFilename)
    : UpdaterExteroceptive(updater_name, minimal_rate, trigger_mode,
                           logFilename),
      EKFUpdaterBase<double, 3, 1>(maximal_mahalanobis_distance) {
  H_(0, MetaState::ORIENTATION_Z) = 1;
  set_log_file_header_({"stamp", "course", "cov_course", "theta", "cov_theta"});
}

//--------------------------------------------------------------------------
void R2WKFUpdaterCourse::update(const Duration& duration,
                                const Observation& current_observation,
                                FSMState& current_fsm_State,
                                MetaState& current_meta_state) {
  assert(current_observation.R() > 0);

  rate_diagnostic_.evaluate(duration);

  switch (current_fsm_State) {
    case FSMState::INIT:
      set_(duration, current_observation, current_meta_state.state,
           current_meta_state.addon);
      break;
    case FSMState::RUNNING:
      if (trigger_mode_ == trigger_mode::ALWAYS) {
        try {
          update_(duration, current_observation, current_meta_state.state,
                  current_meta_state.addon);
        } catch (...) {
          std::cout
              << " FSM : COURSE UPDATE HAS FAILED, RESET AND GO TO INIT MODE"
              << std::endl;
          current_fsm_State = FSMState::INIT;
          current_meta_state.state.reset();
          current_meta_state.addon.reset();
        }
      }
      break;
    default:
      break;
  }
}
//--------------------------------------------------------------------------
void R2WKFUpdaterCourse::update_(const Duration& duration,
                                 const Observation& current_observation,
                                 State& current_state, AddOn& current_add_on) {
  Inn_ = betweenMinusPiAndPi(current_observation.Y() -
                             current_state.X(MetaState::ORIENTATION_Z));

  QInn_ = current_observation.R() +
          current_state.P(MetaState::ORIENTATION_Z, MetaState::ORIENTATION_Z);

  // log
  if (log_file_.is_open()) {
    log_file_ << duration.count() << ",";
    log_file_ << current_observation.Y() << ",";
    log_file_ << current_observation.R() << ",";
    log_file_ << current_state.X(2) << ",";
    log_file_ << current_state.P(2, 2) << ",/n";
  }

  if (!update_state_(current_state)) {
    // TODO(jean) renvoyer un throw
  }

  current_add_on.last_exteroceptive_update.time = duration;
  current_add_on.last_exteroceptive_update.travelled_distance =
      current_add_on.travelled_distance;
}

//--------------------------------------------------------------------------
void R2WKFUpdaterCourse::set_(const Duration& /*duration*/,
                              const Observation& current_observation,
                              State& current_state, AddOn& /*current_add_on*/) {
  current_state.X(MetaState::ORIENTATION_Z) = current_observation.Y();

  current_state.P(MetaState::ORIENTATION_Z, MetaState::ORIENTATION_Z) =
      current_observation.R();

  //  current_add_on.lastExteroceptiveUpdate.time=duration;
  //  current_add_on.lastExteroceptiveUpdate.travelledDistance=current_add_on.travelledDistance;
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
