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
#include "romea_core_localisation/robot_to_robot/particle/results.hpp"

#include <romea_core_common/math/EulerAngles.hpp>

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
R2RPFResults::R2RPFResults(const size_t & number_of_particles)
: R2RResultsBase<R2RPFMetaState>(number_of_particles),
  weight_sum_(0),
  estimate_stamp_(Duration::zero()),
  estimate_(Eigen::Vector3d::Zero()),
  estimate_covariance_stamp_(Duration::zero()),
  estimate_covariance_(Eigen::Matrix3d::Zero()),
  mean_centered_particles_(
    R2RPFMetaState::State::RowMajorMatrix::Zero(STATE_SIZE, number_of_particles))
{
}

//-----------------------------------------------------------------------------
const double & R2RPFResults::get_leader_x() const
{
  lazy_compute_estimate_();
  return estimate_(R2RPFMetaState::LEADER_POSITION_X);
}

//-----------------------------------------------------------------------------
const double & R2RPFResults::get_leader_y() const
{
  lazy_compute_estimate_();
  return estimate_(R2RPFMetaState::LEADER_POSITION_Y);
}

//-----------------------------------------------------------------------------
const double & R2RPFResults::get_leader_orientation() const
{
  lazy_compute_estimate_();
  return estimate_(R2RPFMetaState::LEADER_ORIENTATION_Z);
}

//-----------------------------------------------------------------------------
Eigen::Vector3d R2RPFResults::get_leader_pose() const
{
  lazy_compute_estimate_();
  return estimate_;
}

//-----------------------------------------------------------------------------
Eigen::Matrix3d R2RPFResults::get_leader_pose_covariance() const
{
  lazy_compute_estimate_();
  lazy_compute_estimate_covariance_();
  return estimate_covariance_;
}

//-----------------------------------------------------------------------------
const double & R2RPFResults::get_linear_speed() const
{
  return input.U(LINEAR_SPEED_X_BODY);
}

//-----------------------------------------------------------------------------
const double & R2RPFResults::get_lateral_speed() const
{
  return input.U(LINEAR_SPEED_Y_BODY);
}

//-----------------------------------------------------------------------------
const double & R2RPFResults::get_angular_speed() const
{
  return input.U(ANGULAR_SPEED_Z_BODY);
}

//-----------------------------------------------------------------------------
Eigen::Vector3d R2RPFResults::get_twist() const
{
  return input.U().segment<3>(LINEAR_SPEED_X_BODY);
}

//-----------------------------------------------------------------------------
Eigen::Matrix3d R2RPFResults::get_twist_covariance() const
{
  return input.QU().block<3, 3>(LINEAR_SPEED_X_BODY, LINEAR_SPEED_X_BODY);
}

//-----------------------------------------------------------------------------
const double & R2RPFResults::get_leader_linear_speed() const
{
  return input.U(LEADER_LINEAR_SPEED_X_BODY);
}

//-----------------------------------------------------------------------------
const double & R2RPFResults::get_leader_lateral_speed() const
{
  return input.U(LEADER_LINEAR_SPEED_Y_BODY);
}

//-----------------------------------------------------------------------------
const double & R2RPFResults::get_leader_angular_speed() const
{
  return input.U(LEADER_ANGULAR_SPEED_Z_BODY);
}

//-----------------------------------------------------------------------------
Eigen::Vector3d R2RPFResults::get_leader_twist() const
{
  return input.U().segment<3>(LEADER_LINEAR_SPEED_X_BODY);
}

//-----------------------------------------------------------------------------
Eigen::Matrix3d R2RPFResults::get_leader_twist_covariance() const
{
  return input.QU().block<3, 3>(LEADER_LINEAR_SPEED_X_BODY, LEADER_LINEAR_SPEED_X_BODY);
}

//-----------------------------------------------------------------------------
void R2RPFResults::lazy_compute_estimate_() const
{
  if (estimate_stamp_ != duration_) {
    weight_sum_ = state.weights.sum();
    this->estimate_(0) = (state.particles.row(0) * state.weights).sum() / weight_sum_;
    this->estimate_(1) = (state.particles.row(1) * state.weights).sum() / weight_sum_;
    double C = (state.particles.row(2).cos() * state.weights).sum() / weight_sum_;
    double S = (state.particles.row(2).sin() * state.weights).sum() / weight_sum_;
    this->estimate_(2) = std::atan2(S, C);

    estimate_stamp_ = duration_;
  }
}

//-----------------------------------------------------------------------------
void R2RPFResults::lazy_compute_estimate_covariance_() const
{
  if (estimate_covariance_stamp_ != duration_) {
    for (int i = 0; i < STATE_SIZE; ++i) {
      this->mean_centered_particles_.row(i) = state.particles.row(i) - this->estimate_(i);
    }

    // TODO(jean) à vectoriser
    auto courseRow = this->mean_centered_particles_.row(2);
    for (int n = 0; n < state.particles.cols(); ++n) {
      courseRow(n) = betweenMinusPiAndPi(courseRow(n));
    }

    for (int i = 0; i < STATE_SIZE; ++i) {
      for (int j = i; j < STATE_SIZE; ++j) {
        this->estimate_covariance_(i, j) = this->estimate_covariance_(j, i) =
          (this->mean_centered_particles_.row(i) * this->mean_centered_particles_.row(j) *
           state.weights)
            .sum() /
          weight_sum_;
      }
    }

    estimate_covariance_stamp_ = duration_;
  }
}

//-----------------------------------------------------------------------------
Pose2D R2RPFResults::to_leader_pose2d() const
{
  lazy_compute_estimate_();
  lazy_compute_estimate_covariance_();

  Pose2D pose2d;
  pose2d.position.x() = estimate_(LEADER_POSITION_X);
  pose2d.position.y() = estimate_(LEADER_POSITION_Y);
  pose2d.yaw = estimate_(LEADER_ORIENTATION_Z);
  pose2d.covariance = estimate_covariance_;
  return pose2d;
}

//-----------------------------------------------------------------------------
PoseAndTwist2D R2RPFResults::to_leader_pose_and_body_twist2d() const
{
  lazy_compute_estimate_();
  lazy_compute_estimate_covariance_();

  PoseAndTwist2D poseAndTwist2D;
  poseAndTwist2D.pose.position.x() = estimate_(LEADER_POSITION_X);
  poseAndTwist2D.pose.position.y() = estimate_(LEADER_POSITION_Y);
  poseAndTwist2D.pose.yaw = estimate_(LEADER_ORIENTATION_Z);
  poseAndTwist2D.pose.covariance = estimate_covariance_;
  poseAndTwist2D.twist.linearSpeeds.x() = get_leader_linear_speed();
  poseAndTwist2D.twist.linearSpeeds.y() = get_leader_lateral_speed();
  poseAndTwist2D.twist.angularSpeed = get_leader_angular_speed();
  poseAndTwist2D.twist.covariance = get_leader_twist_covariance();
  return poseAndTwist2D;
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
