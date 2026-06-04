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

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__PREDICTOR_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__PREDICTOR_HPP_

// romea
#include "romea_core_localisation/predictor_base.hpp"
#include "romea_core_localisation/robot_to_world/particle/meta_state.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

class R2WPFPredictor : public PredictorBase<R2WPFMetaState>
{
public:
  using MetaState = R2WPFMetaState;
  using State = R2WPFMetaState::State;
  using Input = R2WPFMetaState::Input;
  using AddOn = R2WPFMetaState::AddOn;
  using RowMajorVector = R2WPFMetaState::State::RowMajorVector;
  using RowMajorMatrix = R2WPFMetaState::State::RowMajorMatrix;

public:
  R2WPFPredictor(
    const Duration & maximal_duration_in_dead_reckoning,
    const double & maximal_travelled_distance_in_dead_reckoning,
    const double & maximal_position_circular_error_probable,
    const size_t & number_of_particles);

private:
  bool stop_(const Duration & duration, const MetaState & state) override;

  void predict_(const MetaState & previous_meta_state, MetaState & current_meta_state) override;

  void reset_(MetaState & metaState) override;

  void predictState_(
    const State & previous_state, const Input & previous_input, State & current_state);

  void predictAddOn_(const AddOn & previous_add_on, AddOn & current_add_on);

  void drawInputs(const Input & previous_input);

private:
  double vxdT_, vydT_;
  RowMajorVector cos_courses_;
  RowMajorVector sin_courses_;
  RowMajorMatrix randomU_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__PREDICTOR_HPP_
