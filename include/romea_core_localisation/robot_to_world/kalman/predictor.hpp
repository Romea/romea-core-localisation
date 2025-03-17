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


#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__PREDICTOR_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__PREDICTOR_HPP_

// romea
#include "romea_core_localisation/predictor_base.hpp"
#include "romea_core_localisation/robot_to_world/kalman/meta_state.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

class R2WKFPredictor : public PredictorBase<R2WKFMetaState>
{
public:
  using MetaState = R2WKFMetaState;
  using State = R2WKFMetaState::State;
  using Input = R2WKFMetaState::Input;
  using AddOn = R2WKFMetaState::AddOn;

public:
  R2WKFPredictor(
    const Duration & maximalDurationInDeadReckoning,
    const double & maximalTravelledDistanceInDeadReckoning,
    const double & maximalPositionCircularErrorProbable);

protected:
  bool stop_(
    const Duration & duration,
    const MetaState & state)override;

  void predict_(
    const MetaState & previousState,
    MetaState & nextState)override;

  void reset_(MetaState & state)override;

  void predictState_(
    const State & previousState,
    const Input & previousInput,
    State & currentState);

  void predictAddOn_(
    const AddOn & previousAddOn,
    AddOn & currentAddOn);

private:
  Eigen::MatrixXd jF_;
  Eigen::MatrixXd jG_;

  double x_, y_, theta_, vx_, vy_, w_;
  double vxdT_, vydT_, wdT_;
  double dT_cos_theta_wdT_;
  double dT_sin_theta_wdT_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif   // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__PREDICTOR_HPP_
