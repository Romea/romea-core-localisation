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
  const std::string & updaterName,
  const double & minimalRate,
  const TriggerMode & triggerMode,
  const size_t & /*numberOfParticles*/,
  const double & /*maximalMahalanobisDistance*/,
  const std::string & logFilename)
: UpdaterExteroceptive(updaterName, minimalRate, triggerMode, logFilename)
{
}


//--------------------------------------------------------------------------
void R2RPFUpdaterLeaderPose::update(
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
bool R2RPFUpdaterLeaderPose::set_(
  const Duration & duration,
  const Observation & currentObservation,
  const Input & currentInput,
  State & currentState,
  AddOn & currentAddOn)
{
  if (!std::isnan(currentInput.U(R2RPFMetaState::LINEAR_SPEED_X_BODY)) &&
    !std::isnan(currentInput.U(R2RPFMetaState::LINEAR_SPEED_Y_BODY)) &&
    !std::isnan(currentInput.U(R2RPFMetaState::ANGULAR_SPEED_Z_BODY)) &&
    !std::isnan(currentInput.U(R2RPFMetaState::LEADER_LINEAR_SPEED_X_BODY)) &&
    !std::isnan(currentInput.U(R2RPFMetaState::LEADER_LINEAR_SPEED_X_BODY)) &&
    !std::isnan(currentInput.U(R2RPFMetaState::LEADER_LINEAR_SPEED_X_BODY)))
  {
    NormalRandomArrayGenerator3D<double> randomGenerator;
    randomGenerator.init(currentObservation.Y(), currentObservation.R());
    randomGenerator.fill(currentState.particles);

    currentAddOn.last_exteroceptive_update.time = duration;
    currentAddOn.last_exteroceptive_update.travelled_distance = currentAddOn.travelled_distance;

    assert(!std::isnan(currentState.particles(0, 0)));
    assert(!std::isnan(currentState.particles(0, 1)));
    assert(!std::isnan(currentState.particles(0, 2)));

    return true;
  } else {
    return false;
  }
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
