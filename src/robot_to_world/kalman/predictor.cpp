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

// romea
#include "romea_core_localisation/robot_to_world/kalman/predictor.hpp"

#include <romea_core_common/math/EulerAngles.hpp>
#include <romea_core_common/math/Matrix.hpp>

namespace romea
{
namespace core
{
namespace localisation
{

//--------------------------------------------------------------------------
R2WKFPredictor::R2WKFPredictor(
  const Duration & maximal_duration_in_dead_reckoning,
  const double & maximal_travelled_distance_in_dead_reckoning,
  const double & maximal_position_circular_error_probable)
: PredictorBase(
    maximal_duration_in_dead_reckoning,
    maximal_travelled_distance_in_dead_reckoning,
    maximal_position_circular_error_probable),
  jF_(Eigen::MatrixXd::Zero(MetaState::STATE_SIZE, MetaState::STATE_SIZE)),
  jG_(Eigen::MatrixXd::Zero(MetaState::STATE_SIZE, MetaState::INPUT_SIZE)),
  x_(0),
  y_(0),
  theta_(0),
  vx_(0),
  vy_(0),
  w_(0),
  vxdT_(0),
  vydT_(0),
  wdT_(0),
  dT_cos_theta_wdT_(0),
  dT_sin_theta_wdT_(0)
{
}

//--------------------------------------------------------------------------
void R2WKFPredictor::predict_(const MetaState & previous_meta_state, MetaState & current_meta_state)
{
  current_meta_state.input = previous_meta_state.input;

  predictState_(previous_meta_state.state, previous_meta_state.input, current_meta_state.state);

  predictAddOn_(previous_meta_state.addon, current_meta_state.addon);

  assert(isPositiveSemiDefiniteMatrix(current_meta_state.state.P()));
  assert(isPositiveSemiDefiniteMatrix(current_meta_state.input.QU()));
}

//-----------------------------------------------------------------------------
void R2WKFPredictor::predictState_(
  const State & previous_state, const Input & previous_input, State & current_state)
{
  // Precompute some data
  x_ = previous_state.X(MetaState::POSITION_X);
  y_ = previous_state.X(MetaState::POSITION_Y);
  theta_ = previous_state.X(MetaState::ORIENTATION_Z);
  vx_ = previous_input.U(MetaState::LINEAR_SPEED_X_BODY);
  vy_ = previous_input.U(MetaState::LINEAR_SPEED_Y_BODY);
  w_ = previous_input.U(MetaState::ANGULAR_SPEED_Z_BODY);

  vxdT_ = vx_ * dt_;
  vydT_ = vy_ * dt_;
  wdT_ = w_ * dt_;

  dT_cos_theta_wdT_ = dt_ * std::cos(theta_ + wdT_);
  dT_sin_theta_wdT_ = dt_ * std::sin(theta_ + wdT_);

  // Predict state vector
  current_state.X(MetaState::POSITION_X) = x_ + vx_ * dT_cos_theta_wdT_ - vy_ * dT_sin_theta_wdT_;
  current_state.X(MetaState::POSITION_Y) = y_ + vx_ * dT_sin_theta_wdT_ + vy_ * dT_cos_theta_wdT_;
  current_state.X(MetaState::ORIENTATION_Z) = between0And2Pi(theta_ + wdT_);

  // Predict state covariance
  jF_(MetaState::POSITION_X, MetaState::POSITION_X) = 1;
  jF_(MetaState::POSITION_X, MetaState::ORIENTATION_Z) =
    -vx_ * dT_sin_theta_wdT_ - vy_ * dT_cos_theta_wdT_;
  jF_(MetaState::POSITION_Y, MetaState::POSITION_Y) = 1;
  jF_(MetaState::POSITION_Y, MetaState::ORIENTATION_Z) =
    vx_ * dT_cos_theta_wdT_ - vy_ * dT_sin_theta_wdT_;
  jF_(MetaState::ORIENTATION_Z, MetaState::ORIENTATION_Z) = 1;

  jG_(MetaState::POSITION_X, MetaState::LINEAR_SPEED_X_BODY) = dT_cos_theta_wdT_;
  jG_(MetaState::POSITION_Y, MetaState::LINEAR_SPEED_X_BODY) = dT_sin_theta_wdT_;
  jG_(MetaState::POSITION_X, MetaState::LINEAR_SPEED_Y_BODY) = -dT_sin_theta_wdT_;
  jG_(MetaState::POSITION_Y, MetaState::LINEAR_SPEED_Y_BODY) = dT_cos_theta_wdT_;

  jG_(MetaState::POSITION_X, MetaState::ANGULAR_SPEED_Z_BODY) =
    -vxdT_ * dT_sin_theta_wdT_ - vydT_ * dT_cos_theta_wdT_;
  jG_(MetaState::POSITION_Y, MetaState::ANGULAR_SPEED_Z_BODY) =
    vxdT_ * dT_cos_theta_wdT_ - vydT_ * dT_sin_theta_wdT_;
  jG_(MetaState::ORIENTATION_Z, MetaState::ANGULAR_SPEED_Z_BODY) = dt_;

  current_state.P().noalias() = jF_ * previous_state.P() * jF_.transpose();
  current_state.P().noalias() += jG_ * previous_input.QU() * jG_.transpose();
}

//-----------------------------------------------------------------------------
void R2WKFPredictor::predictAddOn_(const AddOn & previous_add_on, AddOn & current_add_on)
{
  current_add_on.roll = previous_add_on.roll;
  current_add_on.pitch = previous_add_on.pitch;
  current_add_on.roll_pitch_variance = previous_add_on.roll_pitch_variance;
  current_add_on.last_exteroceptive_update = previous_add_on.last_exteroceptive_update;
  current_add_on.travelled_distance =
    previous_add_on.travelled_distance + std::sqrt(vxdT_ * vxdT_ + vydT_ * vydT_);
}

//-----------------------------------------------------------------------------
bool R2WKFPredictor::stop_(const Duration & duration, const MetaState & metaState)
{
  Duration durationInDeadReckoningMode = duration - metaState.addon.last_exteroceptive_update.time;

  double travelledDistanceInDeadReckoningMode =
    metaState.addon.travelled_distance -
    metaState.addon.last_exteroceptive_update.travelled_distance;

  double positionCircularErrorProbability = std::sqrt(
    metaState.state.P(MetaState::POSITION_X, MetaState::POSITION_X) +
    metaState.state.P(MetaState::POSITION_Y, MetaState::POSITION_Y));

  if (
    positionCircularErrorProbability > maximal_position_circular_error_probable_ ||
    travelledDistanceInDeadReckoningMode > maximal_travelled_distance_in_dead_reckoning_ ||
    durationInDeadReckoningMode > maximal_duration_in_dead_reckoning_) {
    std::cout << " positionCircularErrorProbability " << positionCircularErrorProbability << " "
              << maximal_position_circular_error_probable_ << std::endl;
    std::cout << " travelledDistanceInDeadReckoningMode " << travelledDistanceInDeadReckoningMode
              << " " << maximal_travelled_distance_in_dead_reckoning_ << std::endl;
    std::cout << " durationInDeadReckoningMode " << durationInDeadReckoningMode.count() << " "
              << maximal_duration_in_dead_reckoning_.count() << std::endl;
    return true;
  } else {
    return false;
  }
}

//-----------------------------------------------------------------------------
void R2WKFPredictor::reset_(MetaState & metaState)
{
  metaState.state.reset();
  metaState.addon.reset();
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
