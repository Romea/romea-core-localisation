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

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__PARTICLE__PREDICTOR_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__PARTICLE__PREDICTOR_HPP_

// romea
#include "romea_core_localisation/predictor_base.hpp"
#include "romea_core_localisation/robot_to_robot/particle/meta_state.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

class R2RPFPredictor : public PredictorBase<R2RPFMetaState>
{
public:
  using MetaState = R2RPFMetaState;
  using State = R2RPFMetaState::State;
  using Input = R2RPFMetaState::Input;
  using AddOn = R2RPFMetaState::AddOn;
  using RowMajorVector = R2RPFMetaState::State::RowMajorVector;
  using RowMajorMatrix = R2RPFMetaState::State::RowMajorMatrix;
  using ObservationAgeLimits = PredictorBase<MetaState>::ObservationAgeLimits;

public:
  R2RPFPredictor(
    const size_t & number_of_particles,
    const DeadReckoningLimits & dead_reckoning_limits,
    const ObservationAgeLimits & proprioceptive_observation_age_limits = ObservationAgeLimits());

private:
  void predict_(const MetaState & previous_meta_state, MetaState & current_meta_state) override;

  void predictState_(
    const State & previous_state, const Input & previous_input, State & current_state);

  void predictAddOn_(
    const AddOn & previous_add_on, const State & current_state, AddOn & current_add_on);

  void drawFollowerInputs(const Input & previous_input);

  void drawLeaderInputs(const Input & previous_input);

private:
  RowMajorVector cos_courses_;
  RowMajorVector sin_courses_;

  double wfdT_, vxfdT_, vyfdT_;
  Eigen::Vector3d Uf_;
  Eigen::Matrix3d QUf_;
  Eigen::Vector3d Ufinv_;
  Eigen::Matrix3d QUfinv_;
  RowMajorMatrix randomUfinv_;

  Eigen::Vector3d Ul_;
  Eigen::Matrix3d QUl_;
  RowMajorMatrix randomUl_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__PARTICLE__PREDICTOR_HPP_
