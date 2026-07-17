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
#include "romea_core_localisation/robot_to_robot/particle/predictor.hpp"

#include <romea_core_common/containers/Eigen/EigenContainers.hpp>
#include <romea_core_common/math/EulerAngles.hpp>
#include <romea_core_common/math/Matrix.hpp>
#include <romea_core_common/math/NormalRandomMatrixGenerator.hpp>

namespace romea
{
namespace core
{
namespace localisation
{

//--------------------------------------------------------------------------
R2RPFPredictor::R2RPFPredictor(
  const Duration & maximal_duration_in_dead_reckoning,
  const double & maximal_travelled_distance_in_dead_reckoning,
  const double & maximal_position_circular_error_probable,
  const size_t & number_of_particles)
: PredictorBase<MetaState>(
    maximal_duration_in_dead_reckoning,
    maximal_travelled_distance_in_dead_reckoning,
    maximal_position_circular_error_probable),
  cos_courses_(RowMajorVector::Zero(number_of_particles)),
  sin_courses_(RowMajorVector::Zero(number_of_particles)),
  wfdT_(0),
  vxfdT_(0),
  vyfdT_(0),
  Uf_(Eigen::Vector3d::Zero()),
  QUf_(Eigen::Matrix3d::Zero()),
  Ufinv_(Eigen::Vector3d::Zero()),
  QUfinv_(Eigen::Matrix3d::Zero()),
  randomUfinv_(RowMajorMatrix::Zero(3, number_of_particles)),
  Ul_(Eigen::Vector3d::Zero()),
  QUl_(Eigen::Matrix3d::Zero()),
  randomUl_(RowMajorMatrix::Zero(3, number_of_particles))
{
}

//--------------------------------------------------------------------------
void R2RPFPredictor::predict_(const MetaState & previous_meta_state, MetaState & current_meta_state)
{
  current_meta_state.input = previous_meta_state.input;

  predictState_(previous_meta_state.state, previous_meta_state.input, current_meta_state.state);

  predictAddOn_(previous_meta_state.addon, current_meta_state.state, current_meta_state.addon);

  assert(isPositiveSemiDefiniteMatrix(current_meta_state.input.QU()));
}

//-----------------------------------------------------------------------------
void R2RPFPredictor::drawFollowerInputs(const Input & previous_input)
{
  wfdT_ = previous_input.U(MetaState::ANGULAR_SPEED_Z_BODY) * dt_;
  vxfdT_ = previous_input.U(MetaState::LEADER_LINEAR_SPEED_X_BODY) * dt_;
  vyfdT_ = previous_input.U(MetaState::LEADER_LINEAR_SPEED_Y_BODY) * dt_;

  Uf_ = previous_input.U().segment<3>(MetaState::LINEAR_SPEED_X_BODY) * dt_;

  QUf_ = previous_input.QU().block<3, 3>(
           MetaState::LINEAR_SPEED_X_BODY, MetaState::LINEAR_SPEED_X_BODY) *
         dt_ * dt_;

  Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
  J(0, 0) = -std::cos(wfdT_);
  J(0, 1) = -std::sin(wfdT_);
  J(1, 0) = std::sin(wfdT_);
  J(1, 1) = -std::cos(wfdT_);
  J(2, 2) = -1;

  Ufinv_ = J * Uf_;
  QUfinv_ = J * QUf_ * J.transpose();

  NormalRandomArrayGenerator3D<double> randomGenerator;
  randomGenerator.init(Ufinv_, QUfinv_);
  randomGenerator.fill(randomUfinv_);
}

//-----------------------------------------------------------------------------
void R2RPFPredictor::drawLeaderInputs(const Input & previous_input)
{
  Ul_ = previous_input.U().segment<3>(MetaState::LEADER_LINEAR_SPEED_X_BODY) * dt_;

  QUl_ = previous_input.QU().block<3, 3>(
           MetaState::LEADER_LINEAR_SPEED_X_BODY, MetaState::LEADER_LINEAR_SPEED_X_BODY) *
         dt_ * dt_;

  NormalRandomArrayGenerator3D<double> randomGenerator;
  randomGenerator.init(Ul_, QUl_);
  randomGenerator.fill(randomUl_);
}

//-----------------------------------------------------------------------------
void R2RPFPredictor::predictState_(
  const State & previous_state, const Input & previous_input, State & current_state)
{
  drawFollowerInputs(previous_input);
  drawLeaderInputs(previous_input);

  // predict particles
  current_state.particles.row(MetaState::LEADER_ORIENTATION_Z) =
    previous_state.particles.row(MetaState::LEADER_ORIENTATION_Z) +
    randomUfinv_.row(MetaState::ANGULAR_SPEED_Z_BODY) +
    randomUl_.row(MetaState::ANGULAR_SPEED_Z_BODY);

  cos_courses_ = current_state.particles.row(MetaState::LEADER_ORIENTATION_Z).cos();
  sin_courses_ = current_state.particles.row(MetaState::LEADER_ORIENTATION_Z).sin();

  current_state.particles.row(MetaState::LEADER_POSITION_X) =
    previous_state.particles.row(MetaState::LEADER_POSITION_X) -
    previous_state.particles.row(MetaState::LEADER_POSITION_Y) *
      randomUfinv_.row(MetaState::ANGULAR_SPEED_Z_BODY) +
    randomUfinv_.row(MetaState::LINEAR_SPEED_X_BODY) +
    cos_courses_ * randomUl_.row(MetaState::LINEAR_SPEED_X_BODY) -
    sin_courses_ * randomUl_.row(MetaState::LINEAR_SPEED_Y_BODY);

  current_state.particles.row(MetaState::LEADER_POSITION_Y) =
    previous_state.particles.row(MetaState::LEADER_POSITION_Y) +
    previous_state.particles.row(MetaState::LEADER_POSITION_X) *
      randomUfinv_.row(MetaState::ANGULAR_SPEED_Z_BODY) +
    randomUfinv_.row(MetaState::LINEAR_SPEED_Y_BODY) +
    sin_courses_ * randomUl_.row(MetaState::LINEAR_SPEED_X_BODY) * cos_courses_ *
      randomUl_.row(MetaState::LINEAR_SPEED_Y_BODY);

  current_state.weights = previous_state.weights;
}

//-----------------------------------------------------------------------------
void R2RPFPredictor::predictAddOn_(
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
    double x =
      (current_state.particles.row(MetaState::LEADER_POSITION_X) * current_state.weights).sum();
    double y =
      (current_state.particles.row(MetaState::LEADER_POSITION_Y) * current_state.weights).sum();
    current_add_on.leader_trajectory.append(Eigen::Vector2d(x, y));
  }

  current_add_on.last_exteroceptive_update = previous_add_on.last_exteroceptive_update;
  current_add_on.travelled_distance =
    previous_add_on.travelled_distance + std::sqrt(vxfdT_ * vxfdT_ + vyfdT_ * vyfdT_);
}

//-----------------------------------------------------------------------------
bool R2RPFPredictor::stop_(const Duration & duration, const MetaState & meatastate)
{
  Duration durationInDeadReckoningMode = duration - meatastate.addon.last_exteroceptive_update.time;

  double travelledDistanceInDeadReckoningMode =
    meatastate.addon.travelled_distance -
    meatastate.addon.last_exteroceptive_update.travelled_distance;

  double positionCircularErrorProbability = position_circular_error_probability_(meatastate);

  // std::cout << " particle dr elapsed time " << durationToSecond(duration) <<
  // " " <<
  //       durationToSecond(state.lastExteroceptiveUpdate.time) << " " <<
  //       durationToSecond(
  //   shutoffParameters_.maximal_duration_in_dead_reckoning) << std::endl;
  // std::cout << " particle dr elapsed distance " << state.travelledDistance <<
  // " " <<
  //       state.lastExteroceptiveUpdate.travelledDistance << " " <<
  //       shutoffParameters_.maximal_travelled_distance_in_dead_reckoning <<
  //       std::endl;

  return positionCircularErrorProbability > maximal_position_circular_error_probable_ ||
         travelledDistanceInDeadReckoningMode > maximal_travelled_distance_in_dead_reckoning_ ||
         durationInDeadReckoningMode > maximal_duration_in_dead_reckoning_;
}

//-----------------------------------------------------------------------------
double R2RPFPredictor::position_circular_error_probability_(const MetaState & /*metaState*/) const
{
  return 0;
}

//-----------------------------------------------------------------------------
void R2RPFPredictor::reset_(MetaState & metaState)
{
  metaState.state.reset();
  metaState.addon.reset();
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
