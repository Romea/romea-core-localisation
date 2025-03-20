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


#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__UPDATER_POSITION_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__UPDATER_POSITION_HPP_


// romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_common/math/NormalRandomMatrixGenerator.hpp>
#include <romea_core_filtering/particle/ParticleFilterGaussianUpdaterCore.hpp>

// std
#include <string>

// local
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/observation_position.hpp"
#include "romea_core_localisation/updater_exteroceptive.hpp"
#include "romea_core_localisation/robot_to_world/particle/meta_state.hpp"
#include "romea_core_localisation/robot_to_world/lever_arm_compensation.hpp"


namespace romea
{
namespace core
{
namespace localisation
{

class R2WPFUpdaterPosition : public UpdaterExteroceptive, public PFGaussianUpdaterCore<double, 3, 2>
{
public:
  using Observation = ObservationPosition;
  using MetaState = R2WPFMetaState;
  using State = R2WPFMetaState::State;
  using Input = R2WPFMetaState::Input;
  using AddOn = R2WPFMetaState::AddOn;
  using RowMajorVector = R2WPFMetaState::State::RowMajorVector;
  using RowMajorMatrix = Eigen::Array<double, 2, Eigen::Dynamic, Eigen::RowMajor>;

public:
  R2WPFUpdaterPosition(
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
  void update_(
    const Duration & duration,
    Observation currentObservation,
    State & currentState,
    AddOn & currentAddon);

  bool set_(
    const Duration & duration,
    const Observation & currentObservation,
    const Input & currentInput,
    State & currentState,
    AddOn & currentAddon);

  void computeLevelArms_(
    const Observation & ObservationPosition,
    const AddOn & currentAddon);

  void setParticlePositions_(
    const Observation & ObservationPosition,
    State & currentState);

  void applyLevelArmCompentations_(State & currentState);

private:
  RowMajorMatrix leverArms_;
  RowMajorVector cosCourses_;
  RowMajorVector sinCourses_;
  LeverArmCompensation lever_arm_compensation_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__UPDATER_POSITION_HPP_
