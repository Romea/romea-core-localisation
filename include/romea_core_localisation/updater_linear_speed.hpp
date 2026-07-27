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

#ifndef ROMEA_CORE_LOCALISATION__UPDATER_LINEAR_SPEED_HPP_
#define ROMEA_CORE_LOCALISATION__UPDATER_LINEAR_SPEED_HPP_

// romea
#include <romea_core_common/time/Time.hpp>

// std
#include <array>
#include <string>

// local
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/observation_linear_speed.hpp"
#include "romea_core_localisation/observation_tracking.hpp"
#include "romea_core_localisation/updater_proprioceptive.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

template<class MetaState>
class UpdaterLinearSpeed : public UpdaterProprioceptive
{
public:
  UpdaterLinearSpeed(const std::string & updater_name, const double & minimal_rate)
  : UpdaterProprioceptive(updater_name, minimal_rate)
  {
  }

  using Observation = ObservationLinearSpeed;

  static void update_observation_age_limits(
    ObservationAgeLimits<MetaState::INPUT_SIZE> & observation_age_limits,
    const double & minimal_rate)
  {
    observation_age_limits.update(
      minimal_rate,
      std::array<std::size_t, 2>{MetaState::LINEAR_SPEED_X_BODY, MetaState::LINEAR_SPEED_Y_BODY});
  }

  void update(
    const Duration & duration,
    const Observation & current_observation,
    FSMState & /*current_fsm_State*/,
    MetaState & current_meta_state)
  {
    if (rate_diagnostic_.evaluate(duration) != DiagnosticStatus::OK) {
      return;
    }

    update_input_(current_observation, current_meta_state.input);
    update_tracking_(duration, current_meta_state.addon);
  }

private:
  void update_input_(const Observation & current_observation, typename MetaState::Input & input)
  {
    input.U().template segment<2>(MetaState::LINEAR_SPEED_X_BODY) << current_observation.Y(), 0;

    input.QU().template block<2, 2>(MetaState::LINEAR_SPEED_X_BODY, MetaState::LINEAR_SPEED_X_BODY)
      << current_observation.R(),
      0, 0, 0;
  }

  void update_tracking_(const Duration & duration, typename MetaState::AddOn & add_on)
  {
    add_on.proprioceptive_data_tracking.times[MetaState::LINEAR_SPEED_X_BODY] = duration;
    add_on.proprioceptive_data_tracking.times[MetaState::LINEAR_SPEED_Y_BODY] = duration;
  }
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__UPDATER_LINEAR_SPEED_HPP_
