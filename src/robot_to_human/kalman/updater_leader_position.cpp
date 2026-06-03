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
#include "romea_core_localisation/robot_to_human/kalman/updater_leader_position.hpp"

namespace romea {
namespace core {
namespace localisation {

// TODO(jean)  add update stage
//-----------------------------------------------------------------------------
R2HKFUpdaterLeaderPosition::R2HKFUpdaterLeaderPosition(
    const std::string& updater_name, const double& minimal_rate,
    const trigger_mode& trigger_mode,
    const double& /*maximal_mahalanobis_distance*/,
    const std::string& logFilename)
    : UpdaterExteroceptive(updater_name, minimal_rate, trigger_mode,
                           logFilename) {}

//--------------------------------------------------------------------------
void R2HKFUpdaterLeaderPosition::update(const Duration& duration,
                                        const Observation& current_observation,
                                        FSMState& current_fsm_State,
                                        MetaState& current_meta_state) {
  rate_diagnostic_.evaluate(duration);

  switch (current_fsm_State) {
    case FSMState::INIT:
      if (set_(duration, current_observation, current_meta_state.input,
               current_meta_state.state, current_meta_state.addon)) {
        std::cout << " FSM : INIT DONE, GO TO RUNNING MODE " << std::endl;
        current_fsm_State = FSMState::RUNNING;
      }
      break;
    default:
      break;
  }
}

//-----------------------------------------------------------------------------
bool R2HKFUpdaterLeaderPosition::set_(const Duration& duration,
                                      const Observation& current_observation,
                                      const Input& current_input,
                                      State& current_state,
                                      AddOn& current_add_on) {
  current_add_on.last_exteroceptive_update.time = duration;
  current_add_on.last_exteroceptive_update.travelled_distance =
      current_add_on.travelled_distance;

  if (!std::isnan(current_input.U(MetaState::LINEAR_SPEED_X_BODY)) &&
      !std::isnan(current_input.U(MetaState::LINEAR_SPEED_Y_BODY)) &&
      !std::isnan(current_input.U(MetaState::ANGULAR_SPEED_Z_BODY))) {
    current_state.X().segment<2>(MetaState::LEADER_POSITION_X) =
        current_observation.Y();

    current_state.P().block<2, 2>(MetaState::LEADER_POSITION_X,
                                  MetaState::LEADER_POSITION_X) =
        current_observation.R();

    return true;
  } else {
    return false;
  }
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
