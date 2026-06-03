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

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__PREDICTOR_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__PREDICTOR_HPP_

#include "romea_core_localisation/predictor_base.hpp"
#include "romea_core_localisation/robot_to_human/kalman/meta_state.hpp"

namespace romea {
namespace core {
namespace localisation {

class R2HKFPredictor : public PredictorBase<R2HKFMetaState> {
 public:
  using MetaState = R2HKFMetaState;
  using State = R2HKFMetaState::State;
  using Input = R2HKFMetaState::Input;
  using AddOn = R2HKFMetaState::AddOn;

 public:
  R2HKFPredictor(const Duration& maximal_duration_in_dead_reckoning,
                 const double& maximal_travelled_distance_in_dead_reckoning,
                 const double& maximal_position_circular_error_probable,
                 const double& leaderMotionStd);

  virtual ~R2HKFPredictor() = default;

 private:
  bool stop_(const Duration& duration, const MetaState& state) override;

  void predict_(const MetaState& previous_meta_state,
                MetaState& current_meta_state) override;

  void reset_(MetaState& metaState) override;

 private:
  void predictState_(const State& previous_state, const Input& prviousInput,
                     State& current_state);

  void predictAddOn_(const AddOn& previous_add_on, const State& current_state,
                     AddOn& current_add_on);

 private:
  Eigen::MatrixXd jF_;
  Eigen::MatrixXd jG_;
  Eigen::MatrixXd leaderMotionCovariance_;

  double vx_, vy_, w_;
  double vxdT_, vydT_, wdT_;
  double dT_cos_wdT_;
  double dT_sin_wdT_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__PREDICTOR_HPP_
