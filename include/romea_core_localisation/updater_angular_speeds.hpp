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

#ifndef ROMEA_CORE_LOCALISATION__UPDATER_ANGULAR_SPEEDS_HPP_
#define ROMEA_CORE_LOCALISATION__UPDATER_ANGULAR_SPEEDS_HPP_

// romea
#include <romea_core_common/time/Time.hpp>

// std
#include <string>

// local
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/observation_angular_speeds.hpp"
#include "romea_core_localisation/updater_proprioceptive.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

template<class MetaState>
class UpdaterAngularSpeeds : public UpdaterProprioceptive
{
public:
  using Observation = ObservationAngularSpeeds;

  UpdaterAngularSpeeds(const std::string & updater_name, const double & minimal_rate)
  : UpdaterProprioceptive(updater_name, minimal_rate)
  {
  }

  void update(
    const Duration & duration,
    const Observation & current_observation,
    FSMState & /*current_fsm_State*/,
    MetaState & current_meta_state)
  {
    rate_diagnostic_.evaluate(duration);

    current_meta_state.input.U().template segment<3>(MetaState::ANGULAR_SPEED_X_BODY) =
      current_observation.Y();

    current_meta_state.input.QU().template block<3, 3>(
      MetaState::ANGULAR_SPEED_X_BODY, MetaState::ANGULAR_SPEED_X_BODY) = current_observation.R();
  }
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__UPDATER_ANGULAR_SPEEDS_HPP_
