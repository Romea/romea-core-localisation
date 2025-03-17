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
#include <iostream>
#include <string>

// local
#include "romea_core_localisation/robot_to_robot/kalman/updater_leader_pose.hpp"


namespace romea
{
namespace core
{
namespace localisation
{

// TODO(Jean) add update stage
//-----------------------------------------------------------------------------
R2RKFUpdaterLeaderPose::R2RKFUpdaterLeaderPose(
  const std::string & updaterName,
  const double & minimalRate,
  const TriggerMode & triggerMode,
  const double & /*maximalMahalanobisDistance*/,
  const std::string & logFilename)
: UpdaterExteroceptive(updaterName, minimalRate, triggerMode, logFilename)
{
}

//--------------------------------------------------------------------------
void R2RKFUpdaterLeaderPose::update(
  const Duration & duration,
  const Observation & currentObservation,
  FSMState & currentFSMState,
  MetaState & currentMetaState)
{
  rate_diagnostic_.evaluate(duration);

  switch (currentFSMState) {
    case FSMState::INIT:
      if (set_(
          duration,
          currentObservation,
          currentMetaState.input,
          currentMetaState.state,
          currentMetaState.addon))
      {
        std::cout << " FSM : INIT DONE, GO TO RUNNING MODE " << std::endl;
        currentFSMState = FSMState::RUNNING;
      }
      break;
    default:
      break;
  }
}

//-----------------------------------------------------------------------------
bool R2RKFUpdaterLeaderPose::set_(
  const Duration & duration,
  const Observation & currentObservation,
  const Input & currentInput,
  State & currentState,
  AddOn & currentAddOn)
{
  currentAddOn.last_exteroceptive_update.time = duration;
  currentAddOn.last_exteroceptive_update.travelled_distance = currentAddOn.travelled_distance;

  if (!std::isnan(currentInput.U(R2RKFMetaState::LINEAR_SPEED_X_BODY)) &&
    !std::isnan(currentInput.U(R2RKFMetaState::LINEAR_SPEED_Y_BODY)) &&
    !std::isnan(currentInput.U(R2RKFMetaState::ANGULAR_SPEED_Z_BODY)) &&
    !std::isnan(currentInput.U(R2RKFMetaState::LEADER_LINEAR_SPEED_X_BODY)) &&
    !std::isnan(currentInput.U(R2RKFMetaState::LEADER_LINEAR_SPEED_X_BODY)) &&
    !std::isnan(currentInput.U(R2RKFMetaState::LEADER_LINEAR_SPEED_X_BODY)))
  {
    currentState.X() = currentObservation.Y();
    currentState.P() = currentObservation.R();
    return true;
  } else {
    return false;
  }
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
