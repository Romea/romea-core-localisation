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
#include <romea_core_common/math/EulerAngles.hpp>
#include <romea_core_common/math/Matrix.hpp>

#include "romea_core_localisation/robot_to_robot/kalman/predictor.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
R2RKFPredictor::R2RKFPredictor(
  const DeadReckoningLimits & dead_reckoning_limits,
  const ObservationAgeLimits & proprioceptive_observation_age_limits)
: PredictorBase<MetaState>(dead_reckoning_limits, proprioceptive_observation_age_limits),
  jFl_(Eigen::MatrixXd::Identity(MetaState::STATE_SIZE, MetaState::STATE_SIZE)),
  jGl_(Eigen::MatrixXd::Zero(MetaState::STATE_SIZE, MetaState::INPUT_SIZE)),
  jFf_(Eigen::MatrixXd::Identity(MetaState::STATE_SIZE, MetaState::STATE_SIZE)),
  jGf_(Eigen::MatrixXd::Zero(MetaState::STATE_SIZE, MetaState::INPUT_SIZE)),
  xl_(0),
  yl_(0),
  thetal_(0),
  vxl_(0),
  vyl_(0),
  wl_(0),
  vxldT_(0),
  vyldT_(0),
  wldT_(0),
  dT_cos_thetal_wldT_(0),
  dT_sin_thetal_wldT_(0),
  vxf_(0),
  vyf_(0),
  wf_(0),
  vxfdT_(0),
  vyfdT_(0),
  wfdT_(0),
  dT_cos_wfdT_(0),
  dT_sin_wfdT_(0)
{
}

//-----------------------------------------------------------------------------
void R2RKFPredictor::predict_(
  const R2RKFMetaState & previous_meta_state, R2RKFMetaState & current_meta_state)
{
  current_meta_state.input = previous_meta_state.input;

  predictState_(previous_meta_state.state, previous_meta_state.input, current_meta_state.state);

  predictAddOn_(previous_meta_state.addon, current_meta_state.state, current_meta_state.addon);

  assert(isPositiveSemiDefiniteMatrix(current_meta_state.state.P()));
  assert(isPositiveSemiDefiniteMatrix(current_meta_state.input.QU()));
}

//-----------------------------------------------------------------------------
void R2RKFPredictor::predictState_(
  const State & previous_state, const Input & previous_input, State & current_state)
{
  // Predict state vector according leader displacement
  xl_ = previous_state.X(MetaState::LEADER_POSITION_X);
  yl_ = previous_state.X(MetaState::LEADER_POSITION_Y);
  thetal_ = previous_state.X(MetaState::LEADER_ORIENTATION_Z);

  vxl_ = previous_input.U(MetaState::LEADER_LINEAR_SPEED_X_BODY);
  vxldT_ = vxl_ * dt_;

  vyl_ = previous_input.U(MetaState::LEADER_LINEAR_SPEED_Y_BODY);
  vyldT_ = vyl_ * dt_;

  wl_ = previous_input.U(MetaState::LEADER_ANGULAR_SPEED_Z_BODY);
  wldT_ = wl_ * dt_;

  dT_cos_thetal_wldT_ = dt_ * std::cos(thetal_ + wldT_);
  dT_sin_thetal_wldT_ = dt_ * std::sin(thetal_ + wldT_);

  current_state.X(MetaState::LEADER_POSITION_X) =
    xl_ + vxl_ * dT_cos_thetal_wldT_ - vyl_ * dT_sin_thetal_wldT_;
  current_state.X(MetaState::LEADER_POSITION_Y) =
    yl_ + vxl_ * dT_sin_thetal_wldT_ + vyl_ * dT_cos_thetal_wldT_;
  current_state.X(MetaState::LEADER_ORIENTATION_Z) = betweenMinusPiAndPi(thetal_ + wldT_);

  // Predict state covariance
  jFl_(MetaState::LEADER_POSITION_X, MetaState::LEADER_POSITION_X) = 1;
  jFl_(MetaState::LEADER_POSITION_X, MetaState::LEADER_ORIENTATION_Z) =
    -vxl_ * dT_sin_thetal_wldT_ - vyl_ * dT_cos_thetal_wldT_;
  jFl_(MetaState::LEADER_POSITION_Y, MetaState::LEADER_POSITION_Y) = 1;
  jFl_(MetaState::LEADER_POSITION_Y, MetaState::LEADER_ORIENTATION_Z) =
    vxl_ * dT_cos_thetal_wldT_ - vyl_ * dT_sin_thetal_wldT_;
  jFl_(MetaState::LEADER_ORIENTATION_Z, MetaState::LEADER_ORIENTATION_Z) = 1;

  jGl_(MetaState::LEADER_POSITION_X, MetaState::LEADER_LINEAR_SPEED_X_BODY) = dT_cos_thetal_wldT_;
  jGl_(MetaState::LEADER_POSITION_Y, MetaState::LEADER_LINEAR_SPEED_X_BODY) = dT_sin_thetal_wldT_;
  jGl_(MetaState::LEADER_POSITION_X, MetaState::LEADER_LINEAR_SPEED_Y_BODY) = -dT_sin_thetal_wldT_;
  jGl_(MetaState::LEADER_POSITION_Y, MetaState::LEADER_LINEAR_SPEED_Y_BODY) = dT_cos_thetal_wldT_;

  jGl_(MetaState::LEADER_POSITION_X, MetaState::LEADER_ANGULAR_SPEED_Z_BODY) =
    -vxldT_ * dT_sin_thetal_wldT_ - vyldT_ * dT_cos_thetal_wldT_;
  jGl_(MetaState::LEADER_POSITION_Y, MetaState::LEADER_ANGULAR_SPEED_Z_BODY) =
    vxldT_ * dT_cos_thetal_wldT_ - vyldT_ * dT_sin_thetal_wldT_;
  jGl_(MetaState::LEADER_ORIENTATION_Z, MetaState::LEADER_ANGULAR_SPEED_Z_BODY) = dt_;

  current_state.P().noalias() = jFl_ * previous_state.P() * jFl_.transpose();
  current_state.P().noalias() += jGl_ * previous_input.QU() * jGl_.transpose();

  // Predict state vector according follower displacement
  vxf_ = previous_input.U(MetaState::LINEAR_SPEED_X_BODY);
  vxfdT_ = vxf_ * dt_;

  vyf_ = previous_input.U(MetaState::LINEAR_SPEED_Y_BODY);
  vyfdT_ = vyf_ * dt_;

  wf_ = previous_input.U(MetaState::ANGULAR_SPEED_Z_BODY);
  wfdT_ = wf_ * dt_;

  dT_cos_wfdT_ = dt_ * std::cos(-wfdT_);
  dT_sin_wfdT_ = dt_ * std::sin(-wfdT_);

  Eigen::Matrix2d R = Eigen::Matrix2d(Eigen::Rotation2D<double>(-wfdT_));
  //  Eigen::Vector2d T = - R *Eigen::Vector2d(vxfdT_,vyfdT_);

  jFf_.block<2, 2>(0, 0) = R;
  jGf_.block<2, 2>(0, 0) = -R * dt_;
  jGf_(0, 2) = vxfdT_ * dT_sin_wfdT_ + vyfdT_ * dT_cos_wfdT_;
  jGf_(1, 2) = -vxfdT_ * dT_cos_wfdT_ + vyfdT_ * dT_sin_wfdT_;
  jGf_(2, 2) = -dt_;

  current_state.X() = jFf_ * (current_state.X() - Eigen::Vector3d(vxfdT_, vyfdT_, wfdT_));
  current_state.P() =
    jFf_ * current_state.P() * jFf_.transpose() + jGf_ * previous_input.QU() * jGf_.transpose();
}

//-----------------------------------------------------------------------------
void R2RKFPredictor::predictAddOn_(
  const AddOn & previous_add_on, const State & current_state, AddOn & current_add_on)
{
  Eigen::Matrix2d R = Eigen::Matrix2d(Eigen::Rotation2D<double>(-wfdT_));
  Eigen::Vector2d T = -R * Eigen::Vector2d(vxfdT_, vyfdT_);

  // Predict additional data
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

  current_add_on.travelled_distance =
    previous_add_on.travelled_distance + std::sqrt(vxfdT_ * vxfdT_ + vyfdT_ * vyfdT_);
  current_add_on.dead_reckoning_tracking = previous_add_on.dead_reckoning_tracking;
  current_add_on.proprioceptive_data_tracking = previous_add_on.proprioceptive_data_tracking;
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
