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

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__KALMAN__PREDICTOR_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__KALMAN__PREDICTOR_HPP_

// romea
#include "romea_core_localisation/predictor_base.hpp"
#include "romea_core_localisation/robot_to_robot/kalman/meta_state.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

class R2RKFPredictor : public PredictorBase<R2RKFMetaState>
{
public:
  using MetaState = R2RKFMetaState;
  using State = R2RKFMetaState::State;
  using Input = R2RKFMetaState::Input;
  using AddOn = R2RKFMetaState::AddOn;

public:
  R2RKFPredictor(
    const Duration & maximal_duration_in_dead_reckoning,
    const double & maximal_travelled_distance_in_dead_reckoning,
    const double & maximal_position_circular_error_probable);

private:
  bool stop_(const Duration & duration, const MetaState & metaState) override;

  double position_circular_error_probability_(const MetaState & current_state) const override;

  void predict_(const MetaState & previous_meta_state, MetaState & current_meta_state) override;

  void reset_(R2RKFMetaState & metaState) override;

  void predictState_(
    const State & previous_state, const Input & previous_input, State & current_state);

  void predictAddOn_(
    const AddOn & previous_add_on, const State & current_state, AddOn & current_add_on);

private:
  Eigen::MatrixXd jFl_;
  Eigen::MatrixXd jGl_;
  Eigen::MatrixXd jFf_;
  Eigen::MatrixXd jGf_;

  double xl_, yl_, thetal_;
  double vxl_, vyl_, wl_;
  double vxldT_, vyldT_, wldT_;
  double dT_cos_thetal_wldT_;
  double dT_sin_thetal_wldT_;

  double vxf_, vyf_, wf_;
  double vxfdT_, vyfdT_, wfdT_;
  double dT_cos_wfdT_;
  double dT_sin_wfdT_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__KALMAN__PREDICTOR_HPP_
