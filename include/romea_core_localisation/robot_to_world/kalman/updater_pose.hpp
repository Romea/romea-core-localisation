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

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__UPDATER_POSE_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__UPDATER_POSE_HPP_

// romea
#include <romea_core_common/math/EulerAngles.hpp>
#include <romea_core_common/math/Matrix.hpp>
#include <romea_core_common/time/Time.hpp>
#include <romea_core_filtering/filter/kalman/updater/base/extended.hpp>

// std
#include <string>

// local
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/observation_pose.hpp"
#include "romea_core_localisation/robot_to_world/kalman/meta_state.hpp"
#include "romea_core_localisation/robot_to_world/lever_arm_compensation.hpp"
#include "romea_core_localisation/updater_exteroceptive.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

class R2WKFUpdaterPose : public UpdaterExteroceptive, public EKFUpdaterBase<double, 3, 3>
{
public:
  using Observation = ObservationPose;
  using MetaState = R2WKFMetaState;
  using State = R2WKFMetaState::State;
  using Input = R2WKFMetaState::Input;
  using AddOn = R2WKFMetaState::AddOn;

public:
  R2WKFUpdaterPose(
    const std::string & updater_name,
    const double & minimal_rate,
    const trigger_mode & trigger_mode,
    const double & maximal_mahalanobis_distance,
    const std::string & logFilename);

  void update(
    const Duration & duration,
    const Observation & current_observation,
    FSMState & current_fsm_State,
    MetaState & current_meta_state);

private:
  void update_(
    const Duration & duration,
    const Observation & current_observation,
    State & current_state,
    AddOn & current_add_on);

  bool set_(
    const Duration & duration,
    const ObservationPose & current_observation,
    const Input & current_input,
    State & current_state,
    AddOn & current_add_on);

  //  void applyLeverArmCompensation_(R2WLocalisationKFState & current_state,
  //                                  const Eigen::Vector3d & lever_arm);

  LeverArmCompensation lever_arm_compensation_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__UPDATER_POSE_HPP_
