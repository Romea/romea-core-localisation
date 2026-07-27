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

// eigen
#include <Eigen/Geometry>

// romea
#include <romea_core_common/containers/Eigen/EigenContainers.hpp>
#include <romea_core_common/math/Matrix.hpp>

// std
#include <cmath>

// local
#include "romea_core_localisation/robot_to_human/kalman/predictor.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
R2HKFPredictor::R2HKFPredictor(
  const double & leaderMotionStd,
  const DeadReckoningLimits & dead_reckoning_limits,
  const ObservationAgeLimits & proprioceptive_observation_age_limits)
: PredictorBase(dead_reckoning_limits, proprioceptive_observation_age_limits),
  jF_(Eigen::MatrixXd::Zero(MetaState::STATE_SIZE, MetaState::STATE_SIZE)),
  jG_(Eigen::MatrixXd::Zero(MetaState::STATE_SIZE, MetaState::INPUT_SIZE)),
  leaderMotionCovariance_(Eigen::Matrix2d::Identity() * leaderMotionStd * leaderMotionStd),
  vx_(0),
  vy_(0),
  w_(0),
  vxdT_(0),
  vydT_(0),
  wdT_(0),
  dT_cos_wdT_(0),
  dT_sin_wdT_(0)
{
}

//-----------------------------------------------------------------------------
void R2HKFPredictor::predictState_(
  const State & previous_state, const Input & previous_input, State & current_state)
{
  vx_ = previous_input.U(MetaState::LINEAR_SPEED_X_BODY);
  vxdT_ = vx_ * dt_;

  vy_ = previous_input.U(MetaState::LINEAR_SPEED_Y_BODY);
  vydT_ = vy_ * dt_;

  w_ = previous_input.U(MetaState::ANGULAR_SPEED_Z_BODY);
  wdT_ = w_ * dt_;

  dT_cos_wdT_ = dt_ * std::cos(-wdT_);
  dT_sin_wdT_ = dt_ * std::sin(-wdT_);

  // predict state
  Eigen::Matrix2d R = Eigen::Matrix2d(Eigen::Rotation2D<double>(-wdT_));
  Eigen::Vector2d T = -R * Eigen::Vector2d(vxdT_, vydT_);
  current_state.X() = R * previous_state.X() + T;

  // Predict state covariance
  jF_ = R;
  jG_.block<2, 2>(0, 0) = -R * dt_;
  jG_(0, 2) = (vx_ * dT_sin_wdT_ + vy_ * dT_cos_wdT_) * dt_;
  jG_(1, 2) = (-vx_ * dT_cos_wdT_ + vy_ * dT_sin_wdT_) * dt_;

  current_state.P() =
    jF_ * previous_state.P() * jF_.transpose() + jG_ * previous_input.QU() * jG_.transpose();
  current_state.P() += leaderMotionCovariance_ * dt_ * dt_;
}

//-----------------------------------------------------------------------------
void R2HKFPredictor::predictAddOn_(
  const AddOn & previous_add_on, const State & current_state, AddOn & current_add_on)
{
  Eigen::Matrix2d R = Eigen::Matrix2d(Eigen::Rotation2D<double>(-wdT_));
  Eigen::Vector2d T = -R * Eigen::Vector2d(vxdT_, vydT_);

  transform(previous_add_on.robot_trajectory.get(), current_add_on.robot_trajectory.get(), R, T);

  transform(previous_add_on.leader_trajectory.get(), current_add_on.leader_trajectory.get(), R, T);

  current_add_on.robot_trajectory.ringIndex_ = previous_add_on.robot_trajectory.ringIndex_;
  current_add_on.leader_trajectory.ringIndex_ = previous_add_on.leader_trajectory.ringIndex_;

  if (
    current_add_on.robot_trajectory.size() == 0 ||
    current_add_on.robot_trajectory[0].norm() > 0.1) {
    current_add_on.robot_trajectory.append(Eigen::Vector2d::Zero());
    current_add_on.leader_trajectory.append(current_state.X().head<2>());
  }

  current_add_on.dead_reckoning_tracking = previous_add_on.dead_reckoning_tracking;
  current_add_on.proprioceptive_data_tracking = previous_add_on.proprioceptive_data_tracking;
  current_add_on.travelled_distance =
    previous_add_on.travelled_distance + std::sqrt(vxdT_ * vxdT_ + vydT_ * vydT_);
}

//-----------------------------------------------------------------------------
void R2HKFPredictor::predict_(const MetaState & previous_meta_state, MetaState & current_meta_state)
{
  current_meta_state.input = previous_meta_state.input;

  predictState_(previous_meta_state.state, previous_meta_state.input, current_meta_state.state);

  predictAddOn_(previous_meta_state.addon, current_meta_state.state, current_meta_state.addon);

  assert(isPositiveSemiDefiniteMatrix(current_meta_state.state.P()));
  assert(isPositiveSemiDefiniteMatrix(current_meta_state.input.QU()));
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
