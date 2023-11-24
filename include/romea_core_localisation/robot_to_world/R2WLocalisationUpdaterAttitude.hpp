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


#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__R2WLOCALISATIONUPDATERATTITUDE_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__R2WLOCALISATIONUPDATERATTITUDE_HPP_

// romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_common/math/Algorithm.hpp>

// std
#include <string>

// local
#include "../LocalisationUpdaterProprioceptive.hpp"
#include "../ObservationAttitude.hpp"
#include "../LocalisationFSMState.hpp"


namespace romea
{
namespace core
{


template<class MetaState>
class R2WLocalisationUpdaterAttitude : public LocalisationUpdaterProprioceptive
{
public:
  using Observation = ObservationAttitude;

  R2WLocalisationUpdaterAttitude(
    const std::string & updaterName,
    const double & minimalRate)
  : LocalisationUpdaterProprioceptive(updaterName, minimalRate)
  {
  }

  void update(
    const Duration & duration,
    const Observation & currentObservation,
    LocalisationFSMState & /*currentFSMState*/,
    MetaState & currentMetaState)
  {
    rateDiagnostic_.evaluate(duration);

    assert(near(currentObservation.R(0, 0), currentObservation.R(1, 1)));
    currentMetaState.addon.roll = currentObservation.Y(ObservationAttitude::ROLL);
    currentMetaState.addon.pitch = currentObservation.Y(ObservationAttitude::PITCH);
    currentMetaState.addon.rollPitchVariance = currentObservation.R(
      ObservationAttitude::ROLL,
      ObservationAttitude::ROLL);
  }
};

}  // namespace core
}  // namespace romea


#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__R2WLOCALISATIONUPDATERATTITUDE_HPP_
