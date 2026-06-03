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

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__UPDATER_RANGE_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__UPDATER_RANGE_HPP_

// romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_filtering/filter/particle/updater/base/gaussian.hpp>

// std
#include <string>

// local
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/observation_range.hpp"
#include "romea_core_localisation/robot_to_world/lever_arm_compensation.hpp"
#include "romea_core_localisation/robot_to_world/particle/meta_state.hpp"
#include "romea_core_localisation/updater_exteroceptive.hpp"

namespace romea {
namespace core {
namespace localisation {

class R2WPFUpdaterRange : public UpdaterExteroceptive,
                          public PFGaussianUpdaterBase<double, 3, 1> {
 public:
  using Observation = ObservationRange;
  using MetaState = R2WPFMetaState;
  using State = R2WPFMetaState::State;
  using Input = R2WPFMetaState::Input;
  using AddOn = R2WPFMetaState::AddOn;
  using RowMajorVector = R2WPFMetaState::State::RowMajorVector;

 public:
  R2WPFUpdaterRange(const std::string& updater_name, const double& minimal_rate,
                    const trigger_mode& trigger_mode,
                    const size_t& number_of_particles,
                    const double& maximal_mahalanobis_distance,
                    const std::string& logFilename);

  void update(const Duration& duration, const Observation& current_observation,
              FSMState& current_fsm_State, MetaState& current_state);

 protected:
  void update_(const Duration& duration, Observation current_observation,
               State& current_state, AddOn& current_add_on);

 protected:
  RowMajorVector cos_courses_;
  RowMajorVector sin_courses_;
  LeverArmCompensation lever_arm_compensation_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__UPDATER_RANGE_HPP_
