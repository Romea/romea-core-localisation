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

#include "romea_core_localisation/robot_to_world/particle/meta_state.hpp"

#include "romea_core_common/math/EulerAngles.hpp"
#include "romea_core_common/math/Matrix.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//--------------------------------------------------------------------------
R2WPFMetaState::R2WPFMetaState(const size_t & number_of_particles)
: R2WMetaStateBase(), state(number_of_particles)
{
}

//--------------------------------------------------------------------------
R2WPFMetaStateToResults::R2WPFMetaStateToResults(const size_t & number_of_particles)
: mean_centered_particles_(RowMajorMatrix::Zero(R2WPFMetaState::STATE_SIZE, number_of_particles))
{
}

//--------------------------------------------------------------------------
R2WResults R2WPFMetaStateToResults::convert(const R2WPFMetaState & meta_state) const
{
  const auto estimate = compute_estimate_(meta_state);
  const auto estimate_covariance = compute_estimate_covariance_(meta_state, estimate);
  R2WResults results;

  results.robot_pose.position.x() = estimate(R2WPFMetaState::POSITION_X);
  results.robot_pose.position.y() = estimate(R2WPFMetaState::POSITION_Y);
  results.robot_pose.orientation.x() = meta_state.addon.roll;
  results.robot_pose.orientation.y() = meta_state.addon.pitch;
  results.robot_pose.orientation.z() = estimate(R2WPFMetaState::ORIENTATION_Z);
  results.robot_pose.covariance = toSe3Covariance(estimate_covariance);
  results.robot_pose.covariance(3, 3) = meta_state.addon.roll_pitch_variance;
  results.robot_pose.covariance(4, 4) = meta_state.addon.roll_pitch_variance;

  results.robot_twist.linearSpeeds.x() =
    meta_state.input.U(R2WPFMetaState::LINEAR_SPEED_X_BODY);
  results.robot_twist.linearSpeeds.y() =
    meta_state.input.U(R2WPFMetaState::LINEAR_SPEED_Y_BODY);
  results.robot_twist.angularSpeeds.z() =
    meta_state.input.U(R2WPFMetaState::ANGULAR_SPEED_Z_BODY);
  results.robot_twist.covariance = toSe3Covariance(meta_state.input.QU());

  return results;
}

//--------------------------------------------------------------------------
Eigen::Vector3d R2WPFMetaStateToResults::compute_estimate_(
  const R2WPFMetaState & meta_state) const
{
  const double weight_sum = meta_state.state.weights.sum();
  Eigen::Vector3d estimate;
  estimate(0) = (meta_state.state.particles.row(0) * meta_state.state.weights).sum() / weight_sum;
  estimate(1) = (meta_state.state.particles.row(1) * meta_state.state.weights).sum() / weight_sum;
  const double C =
    (meta_state.state.particles.row(2).cos() * meta_state.state.weights).sum() / weight_sum;
  const double S =
    (meta_state.state.particles.row(2).sin() * meta_state.state.weights).sum() / weight_sum;
  estimate(2) = std::atan2(S, C);
  return estimate;
}

//--------------------------------------------------------------------------
Eigen::Matrix3d R2WPFMetaStateToResults::compute_estimate_covariance_(
  const R2WPFMetaState & meta_state,
  const Eigen::Vector3d & estimate) const
{
  const double weight_sum = meta_state.state.weights.sum();
  for (int i = 0; i < R2WPFMetaState::STATE_SIZE; ++i) {
    mean_centered_particles_.row(i) = meta_state.state.particles.row(i) - estimate(i);
  }

  auto course_row = mean_centered_particles_.row(2);
  for (int n = 0; n < meta_state.state.particles.cols(); ++n) {
    course_row(n) = betweenMinusPiAndPi(course_row(n));
  }

  Eigen::Matrix3d covariance;
  for (int i = 0; i < R2WPFMetaState::STATE_SIZE; ++i) {
    for (int j = i; j < R2WPFMetaState::STATE_SIZE; ++j) {
      covariance(i, j) = covariance(j, i) =
        (mean_centered_particles_.row(i) * mean_centered_particles_.row(j) *
         meta_state.state.weights).sum() / weight_sum;
    }
  }
  return covariance;
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
