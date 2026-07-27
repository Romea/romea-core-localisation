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
#include <romea_core_common/math/NormalRandomMatrixGenerator.hpp>

// std
#include <string>

// local
#include "romea_core_localisation/robot_to_robot/particle/updater_leader_pose.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

// TODO(jean) add update stage
//-----------------------------------------------------------------------------
R2RPFUpdaterLeaderPose::R2RPFUpdaterLeaderPose(
  const std::string & updater_name,
  const double & minimal_rate,
  const trigger_mode & trigger_mode,
  const size_t & /*number_of_particles*/,
  const double & /*maximal_mahalanobis_distance*/)
: UpdaterExteroceptive(updater_name, minimal_rate, trigger_mode)
{
}

//--------------------------------------------------------------------------
void R2RPFUpdaterLeaderPose::update(
  const Duration & duration,
  const Observation & current_observation,
  FSMState & current_fsm_State,
  MetaState & current_meta_state)
{
  rate_diagnostic_.evaluate(duration);

  switch (current_fsm_State) {
    case FSMState::INIT:
      if (set_(
            duration,
            current_observation,
            current_meta_state.input,
            current_meta_state.state,
            current_meta_state.addon)) {
        const auto previous_fsm_state = current_fsm_State;
        current_fsm_State = FSMState::RUNNING;
        notify_fsm_event_(
          previous_fsm_state,
          current_fsm_State,
          "INIT DONE, GO TO RUNNING MODE");
      }
      break;
    default:
      break;
  }
}

//-----------------------------------------------------------------------------
bool R2RPFUpdaterLeaderPose::set_(
  const Duration & duration,
  const Observation & current_observation,
  const Input & current_input,
  State & current_state,
  AddOn & current_add_on)
{
  if (
    !std::isnan(current_input.U(R2RPFMetaState::LINEAR_SPEED_X_BODY)) &&
    !std::isnan(current_input.U(R2RPFMetaState::LINEAR_SPEED_Y_BODY)) &&
    !std::isnan(current_input.U(R2RPFMetaState::ANGULAR_SPEED_Z_BODY)) &&
    !std::isnan(current_input.U(R2RPFMetaState::LEADER_LINEAR_SPEED_X_BODY)) &&
    !std::isnan(current_input.U(R2RPFMetaState::LEADER_LINEAR_SPEED_X_BODY)) &&
    !std::isnan(current_input.U(R2RPFMetaState::LEADER_LINEAR_SPEED_X_BODY))) {
    NormalRandomArrayGenerator3D<double> randomGenerator;
    randomGenerator.init(current_observation.Y(), current_observation.R());
    randomGenerator.fill(current_state.particles);

    current_add_on.dead_reckoning_tracking.start_time = duration;
    current_add_on.dead_reckoning_tracking.start_travelled_distance = current_add_on.travelled_distance;

    assert(!std::isnan(current_state.particles(0, 0)));
    assert(!std::isnan(current_state.particles(0, 1)));
    assert(!std::isnan(current_state.particles(0, 2)));

    return true;
  } else {
    return false;
  }
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
