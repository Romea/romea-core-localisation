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
  const Duration & maximalDurationInDeadReckoning,
  const double & maximalTravelledDistanceInDeadReckoning,
  const double & maximalPositionCircularErrorProbable,
  const double & leaderMotionStd)
: PredictorBase(maximalDurationInDeadReckoning,
    maximalTravelledDistanceInDeadReckoning,
    maximalPositionCircularErrorProbable),
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
  const State & previousState,
  const Input & previousInput,
  State & currentState)
{
  vx_ = previousInput.U(MetaState::LINEAR_SPEED_X_BODY);
  vxdT_ = vx_ * dt_;

  vy_ = previousInput.U(MetaState::LINEAR_SPEED_Y_BODY);
  vydT_ = vy_ * dt_;

  w_ = previousInput.U(MetaState::ANGULAR_SPEED_Z_BODY);
  wdT_ = w_ * dt_;

  dT_cos_wdT_ = dt_ * std::cos(-wdT_);
  dT_sin_wdT_ = dt_ * std::sin(-wdT_);

  // predict state
  Eigen::Matrix2d R = Eigen::Matrix2d(Eigen::Rotation2D<double>(-wdT_));
  Eigen::Vector2d T = -R * Eigen::Vector2d(vxdT_, vydT_);
  currentState.X() = R * previousState.X() + T;

  // Predict state covariance
  jF_ = R;
  jG_.block<2, 2>(0, 0) = -R * dt_;
  jG_(0, 2) = (vx_ * dT_sin_wdT_ + vy_ * dT_cos_wdT_) * dt_;
  jG_(1, 2) = (-vx_ * dT_cos_wdT_ + vy_ * dT_sin_wdT_) * dt_;

  currentState.P() = jF_ * previousState.P() * jF_.transpose() + jG_ * previousInput.QU() *
    jG_.transpose();
  currentState.P() += leaderMotionCovariance_ * dt_ * dt_;
}

//-----------------------------------------------------------------------------
void R2HKFPredictor::predictAddOn_(
  const AddOn & previousAddOn,
  const State & currentState,
  AddOn & currentAddOn)
{
  Eigen::Matrix2d R = Eigen::Matrix2d(Eigen::Rotation2D<double>(-wdT_));
  Eigen::Vector2d T = -R * Eigen::Vector2d(vxdT_, vydT_);

  transform(
    previousAddOn.robot_trajectory.get(),
    currentAddOn.robot_trajectory.get(),
    R, T);

  transform(
    previousAddOn.leader_trajectory.get(),
    currentAddOn.leader_trajectory.get(),
    R, T);

  currentAddOn.robot_trajectory.ringIndex_ = previousAddOn.robot_trajectory.ringIndex_;
  currentAddOn.leader_trajectory.ringIndex_ = previousAddOn.leader_trajectory.ringIndex_;

  if (currentAddOn.robot_trajectory.size() == 0 ||
    currentAddOn.robot_trajectory[0].norm() > 0.1)
  {
    currentAddOn.robot_trajectory.append(Eigen::Vector2d::Zero());
    currentAddOn.leader_trajectory.append(currentState.X().head<2>());
  }

  currentAddOn.last_exteroceptive_update = previousAddOn.last_exteroceptive_update;
  currentAddOn.travelled_distance = previousAddOn.travelled_distance +
    std::sqrt(vxdT_ * vxdT_ + vydT_ * vydT_);
}


//-----------------------------------------------------------------------------
void R2HKFPredictor::predict_(
  const MetaState & previousMetaState,
  MetaState & currentMetaState)
{
  currentMetaState.input = previousMetaState.input;

  predictState_(
    previousMetaState.state,
    previousMetaState.input,
    currentMetaState.state);

  predictAddOn_(
    previousMetaState.addon,
    currentMetaState.state,
    currentMetaState.addon);

  assert(isPositiveSemiDefiniteMatrix(currentMetaState.state.P()));
  assert(isPositiveSemiDefiniteMatrix(currentMetaState.input.QU()));
}

//-----------------------------------------------------------------------------
bool R2HKFPredictor::stop_(
  const Duration & duration,
  const MetaState & metaState)
{
  Duration durationInDeadReckoningMode = duration - metaState.addon.last_exteroceptive_update.time;

  double travelledDistanceInDeadReckoningMode = metaState.addon.travelled_distance -
    metaState.addon.last_exteroceptive_update.travelled_distance;

  double positionCircularErrorProbability =
    std::sqrt(
    metaState.state.P(MetaState::LEADER_POSITION_X, MetaState::LEADER_POSITION_X) +
    metaState.state.P(MetaState::LEADER_POSITION_Y, MetaState::LEADER_POSITION_Y));

  return positionCircularErrorProbability > maximalPositionCircularErrorProbable_ ||
         travelledDistanceInDeadReckoningMode > maximalTravelledDistanceInDeadReckoning_ ||
         durationInDeadReckoningMode > maximalDurationInDeadReckoning_;
}

//-----------------------------------------------------------------------------
void R2HKFPredictor::reset_(R2HKFMetaState & metaState)
{
  metaState.state.reset();
  metaState.addon.reset();
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
