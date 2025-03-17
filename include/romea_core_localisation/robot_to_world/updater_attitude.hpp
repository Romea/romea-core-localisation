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


#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__UPDATER_ATTITUDE_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__UPDATER_ATTITUDE_HPP_

// romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_common/math/Algorithm.hpp>

// std
#include <string>

// local
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/observation_attitude.hpp"
#include "romea_core_localisation/updater_proprioceptive.hpp"


namespace romea
{
namespace core
{
namespace localisation
{

template<class MetaState>
class R2WUpdaterAttitude : public UpdaterProprioceptive
{
public:
  using Observation = ObservationAttitude;

  R2WUpdaterAttitude(
    const std::string & updaterName,
    const double & minimalRate)
  : UpdaterProprioceptive(updaterName, minimalRate)
  {
  }

  void update(
    const Duration & duration,
    const Observation & currentObservation,
    FSMState & /*currentFSMState*/,
    MetaState & currentMetaState)
  {
    rate_diagnostic_.evaluate(duration);

    assert(near(currentObservation.R(0, 0), currentObservation.R(1, 1)));
    currentMetaState.addon.roll = currentObservation.Y(ObservationAttitude::ROLL);
    currentMetaState.addon.pitch = currentObservation.Y(ObservationAttitude::PITCH);
    currentMetaState.addon.roll_pitch_variance = currentObservation.R(
      ObservationAttitude::ROLL,
      ObservationAttitude::ROLL);
  }
};

}  // namespace localisation
}  // namespace core
}  // namespace romea


#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__UPDATER_ATTITUDE_HPP_
