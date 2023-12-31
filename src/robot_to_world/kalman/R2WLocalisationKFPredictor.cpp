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


// romea
#include "romea_core_localisation/robot_to_world/kalman/R2WLocalisationKFPredictor.hpp"
#include <romea_core_common/math/EulerAngles.hpp>
#include <romea_core_common/math/Matrix.hpp>

namespace romea
{
namespace core
{

//--------------------------------------------------------------------------
R2WLocalisationKFPredictor::R2WLocalisationKFPredictor(
  const Duration & maximalDurationInDeadReckoning,
  const double & maximalTravelledDistanceInDeadReckoning,
  const double & maximalPositionCircularErrorProbable)
: LocalisationPredictor(maximalDurationInDeadReckoning,
    maximalTravelledDistanceInDeadReckoning,
    maximalPositionCircularErrorProbable),
  jF_(Eigen::MatrixXd::Zero(MetaState::STATE_SIZE, MetaState::STATE_SIZE)),
  jG_(Eigen::MatrixXd::Zero(MetaState::STATE_SIZE, MetaState::INPUT_SIZE)),
  x_(0), y_(0), theta_(0), vx_(0), vy_(0), w_(0),
  vxdT_(0), vydT_(0), wdT_(0),
  dT_cos_theta_wdT_(0),
  dT_sin_theta_wdT_(0)
{
}

//--------------------------------------------------------------------------
void R2WLocalisationKFPredictor::predict_(
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
    currentMetaState.addon);

  assert(isPositiveSemiDefiniteMatrix(currentMetaState.state.P()));
  assert(isPositiveSemiDefiniteMatrix(currentMetaState.input.QU()));
}


//-----------------------------------------------------------------------------
void R2WLocalisationKFPredictor::predictState_(
  const State & previousState,
  const Input & previousInput,
  State & currentState)
{
  // Precompute some data
  x_ = previousState.X(MetaState::POSITION_X);
  y_ = previousState.X(MetaState::POSITION_Y);
  theta_ = previousState.X(MetaState::ORIENTATION_Z);
  vx_ = previousInput.U(MetaState::LINEAR_SPEED_X_BODY);
  vy_ = previousInput.U(MetaState::LINEAR_SPEED_Y_BODY);
  w_ = previousInput.U(MetaState::ANGULAR_SPEED_Z_BODY);

  vxdT_ = vx_ * dt_;
  vydT_ = vy_ * dt_;
  wdT_ = w_ * dt_;

  dT_cos_theta_wdT_ = dt_ * std::cos(theta_ + wdT_);
  dT_sin_theta_wdT_ = dt_ * std::sin(theta_ + wdT_);

  // Predict state vector
  currentState.X(MetaState::POSITION_X) = x_ + vx_ * dT_cos_theta_wdT_ - vy_ * dT_sin_theta_wdT_;
  currentState.X(MetaState::POSITION_Y) = y_ + vx_ * dT_sin_theta_wdT_ + vy_ * dT_cos_theta_wdT_;
  currentState.X(MetaState::ORIENTATION_Z) = between0And2Pi(theta_ + wdT_);

  // Predict state covariance
  jF_(
    MetaState::POSITION_X,
    MetaState::POSITION_X) = 1;
  jF_(
    MetaState::POSITION_X,
    MetaState::ORIENTATION_Z) = -vx_ * dT_sin_theta_wdT_ - vy_ * dT_cos_theta_wdT_;
  jF_(
    MetaState::POSITION_Y,
    MetaState::POSITION_Y) = 1;
  jF_(
    MetaState::POSITION_Y,
    MetaState::ORIENTATION_Z) = vx_ * dT_cos_theta_wdT_ - vy_ * dT_sin_theta_wdT_;
  jF_(
    MetaState::ORIENTATION_Z,
    MetaState::ORIENTATION_Z) = 1;


  jG_(
    MetaState::POSITION_X,
    MetaState::LINEAR_SPEED_X_BODY) = dT_cos_theta_wdT_;
  jG_(
    MetaState::POSITION_Y,
    MetaState::LINEAR_SPEED_X_BODY) = dT_sin_theta_wdT_;
  jG_(
    MetaState::POSITION_X,
    MetaState::LINEAR_SPEED_Y_BODY) = -dT_sin_theta_wdT_;
  jG_(
    MetaState::POSITION_Y,
    MetaState::LINEAR_SPEED_Y_BODY) = dT_cos_theta_wdT_;

  jG_(
    MetaState::POSITION_X,
    MetaState::ANGULAR_SPEED_Z_BODY) = -vxdT_ * dT_sin_theta_wdT_ - vydT_ * dT_cos_theta_wdT_;
  jG_(
    MetaState::POSITION_Y,
    MetaState::ANGULAR_SPEED_Z_BODY) = vxdT_ * dT_cos_theta_wdT_ - vydT_ * dT_sin_theta_wdT_;
  jG_(
    MetaState::ORIENTATION_Z,
    MetaState::ANGULAR_SPEED_Z_BODY) = dt_;

  currentState.P().noalias() = jF_ * previousState.P() * jF_.transpose();
  currentState.P().noalias() += jG_ * previousInput.QU() * jG_.transpose();
}

//-----------------------------------------------------------------------------
void R2WLocalisationKFPredictor::predictAddOn_(
  const AddOn & previousAddOn,
  AddOn & currentAddOn)
{
  currentAddOn.roll = previousAddOn.roll;
  currentAddOn.pitch = previousAddOn.pitch;
  currentAddOn.rollPitchVariance = previousAddOn.rollPitchVariance;
  currentAddOn.lastExteroceptiveUpdate = previousAddOn.lastExteroceptiveUpdate;
  currentAddOn.travelledDistance = previousAddOn.travelledDistance +
    std::sqrt(vxdT_ * vxdT_ + vydT_ * vydT_);
}


//-----------------------------------------------------------------------------
bool R2WLocalisationKFPredictor::stop_(
  const Duration & duration,
  const MetaState & metaState)
{
  Duration durationInDeadReckoningMode = duration - metaState.addon.lastExteroceptiveUpdate.time;

  double travelledDistanceInDeadReckoningMode =
    metaState.addon.travelledDistance - metaState.addon.lastExteroceptiveUpdate.travelledDistance;


  double positionCircularErrorProbability = std::sqrt(
    metaState.state.P(
      MetaState::POSITION_X,
      MetaState::POSITION_X) +
    metaState.state.P(
      MetaState::POSITION_Y,
      MetaState::POSITION_Y));

  if (positionCircularErrorProbability > maximalPositionCircularErrorProbable_ ||
    travelledDistanceInDeadReckoningMode > maximalTravelledDistanceInDeadReckoning_ ||
    durationInDeadReckoningMode > maximalDurationInDeadReckoning_)
  {
    std::cout << " positionCircularErrorProbability " <<
      positionCircularErrorProbability << " " <<
      maximalPositionCircularErrorProbable_ << std::endl;
    std::cout << " travelledDistanceInDeadReckoningMode " <<
      travelledDistanceInDeadReckoningMode << " " <<
      maximalTravelledDistanceInDeadReckoning_ << std::endl;
    std::cout << " durationInDeadReckoningMode " <<
      durationInDeadReckoningMode.count() << " " <<
      maximalDurationInDeadReckoning_.count() << std::endl;
    return true;
  } else {
    return false;
  }
}


//-----------------------------------------------------------------------------
void R2WLocalisationKFPredictor::reset_(MetaState & metaState)
{
  metaState.state.reset();
  metaState.addon.reset();
}

}  // namespace core
}  // namespace romea
