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

#include "romea_core_localisation/robot_to_robot/particle/meta_state.hpp"

#include "romea_core_common/math/EulerAngles.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//--------------------------------------------------------------------------
R2RPFMetaState::R2RPFMetaState(const size_t & number_of_particles)
: R2RMetaStateBase(), state(number_of_particles)
{
}

//--------------------------------------------------------------------------
R2RPFMetaStateToResults::R2RPFMetaStateToResults(const size_t & number_of_particles)
: mean_centered_particles_(RowMajorMatrix::Zero(R2RPFMetaState::STATE_SIZE, number_of_particles))
{
}

//--------------------------------------------------------------------------
R2RResults R2RPFMetaStateToResults::convert(const R2RPFMetaState & meta_state) const
{
  const auto estimate = compute_estimate_(meta_state);
  const auto estimate_covariance = compute_estimate_covariance_(meta_state, estimate);
  R2RResults results;

  results.leader_pose_and_twist.pose.position.x() = estimate(R2RPFMetaState::LEADER_POSITION_X);
  results.leader_pose_and_twist.pose.position.y() = estimate(R2RPFMetaState::LEADER_POSITION_Y);
  results.leader_pose_and_twist.pose.yaw = estimate(R2RPFMetaState::LEADER_ORIENTATION_Z);
  results.leader_pose_and_twist.pose.covariance = estimate_covariance;

  results.follower_twist.linearSpeeds.x() =
    meta_state.input.U(R2RPFMetaState::LINEAR_SPEED_X_BODY);
  results.follower_twist.linearSpeeds.y() =
    meta_state.input.U(R2RPFMetaState::LINEAR_SPEED_Y_BODY);
  results.follower_twist.angularSpeed =
    meta_state.input.U(R2RPFMetaState::ANGULAR_SPEED_Z_BODY);
  results.follower_twist.covariance =
    meta_state.input.QU().block<3, 3>(
    R2RPFMetaState::LINEAR_SPEED_X_BODY, R2RPFMetaState::LINEAR_SPEED_X_BODY);

  results.leader_pose_and_twist.twist.linearSpeeds.x() =
    meta_state.input.U(R2RPFMetaState::LEADER_LINEAR_SPEED_X_BODY);
  results.leader_pose_and_twist.twist.linearSpeeds.y() =
    meta_state.input.U(R2RPFMetaState::LEADER_LINEAR_SPEED_Y_BODY);
  results.leader_pose_and_twist.twist.angularSpeed =
    meta_state.input.U(R2RPFMetaState::LEADER_ANGULAR_SPEED_Z_BODY);
  results.leader_pose_and_twist.twist.covariance =
    meta_state.input.QU().block<3, 3>(
    R2RPFMetaState::LEADER_LINEAR_SPEED_X_BODY,
    R2RPFMetaState::LEADER_LINEAR_SPEED_X_BODY);

  return results;
}

//--------------------------------------------------------------------------
Eigen::Vector3d R2RPFMetaStateToResults::compute_estimate_(
  const R2RPFMetaState & meta_state) const
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
Eigen::Matrix3d R2RPFMetaStateToResults::compute_estimate_covariance_(
  const R2RPFMetaState & meta_state,
  const Eigen::Vector3d & estimate) const
{
  const double weight_sum = meta_state.state.weights.sum();
  for (int i = 0; i < R2RPFMetaState::STATE_SIZE; ++i) {
    mean_centered_particles_.row(i) = meta_state.state.particles.row(i) - estimate(i);
  }

  auto course_row = mean_centered_particles_.row(2);
  for (int n = 0; n < meta_state.state.particles.cols(); ++n) {
    course_row(n) = betweenMinusPiAndPi(course_row(n));
  }

  Eigen::Matrix3d covariance;
  for (int i = 0; i < R2RPFMetaState::STATE_SIZE; ++i) {
    for (int j = i; j < R2RPFMetaState::STATE_SIZE; ++j) {
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
