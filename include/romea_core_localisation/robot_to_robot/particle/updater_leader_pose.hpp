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


#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__PARTICLE__UPDATER_LEADER_POSE_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__PARTICLE__UPDATER_LEADER_POSE_HPP_

// romea
#include <romea_core_common/time/Time.hpp>

// std
#include <string>

// local
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/observation_pose.hpp"
#include "romea_core_localisation/updater_exteroceptive.hpp"
#include "romea_core_localisation/robot_to_robot/particle/meta_state.hpp"

namespace romea
{
namespace core
{
namespace localisation
{
  
class R2RPFUpdaterLeaderPose : public UpdaterExteroceptive
{
public:
  using Observation = ObservationPose;
  using MetaState = R2RPFMetaState;
  using State = R2RPFMetaState::State;
  using Input = R2RPFMetaState::Input;
  using AddOn = R2RPFMetaState::AddOn;

public:
  R2RPFUpdaterLeaderPose(
    const std::string & updaterName,
    const double & minimalRate,
    const TriggerMode & triggerMode,
    const size_t & numberOfParticles,
    const double & maximalMahalanobisDistance,
    const std::string & logFilename);

  void update(
    const Duration & duration,
    const Observation & currentObservation,
    FSMState & currentFSMState,
    MetaState & currentMetaState);

private:
  bool set_(
    const Duration & duration,
    const Observation & currentObservation,
    const Input & currentInput,
    State & currentState,
    AddOn & currentAddOn);
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__PARTICLE__UPDATER_LEADER_POSE_HPP_
