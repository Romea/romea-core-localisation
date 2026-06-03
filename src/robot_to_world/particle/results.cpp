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
#include "romea_core_localisation/robot_to_world/particle/results.hpp"

#include <romea_core_common/math/EulerAngles.hpp>

namespace romea {
namespace core {
namespace localisation {

//-----------------------------------------------------------------------------
R2WPFResults::R2WPFResults(const size_t& number_of_particles)
    : R2WResultsBase<R2WPFMetaState>(number_of_particles),
      weight_sum_(0),
      estimate_stamp_(Duration::zero()),
      estimate_(Eigen::Vector3d::Zero()),
      estimate_covariance_stamp_(Duration::zero()),
      estimate_covariance_(Eigen::Matrix3d::Zero()),
      mean_centered_particles_(
          RowMajorMatrix::Zero(STATE_SIZE, number_of_particles)) {}

//-----------------------------------------------------------------------------
R2WPFResults::~R2WPFResults() {}

//-----------------------------------------------------------------------------
void R2WPFResults::reset(const Duration& duration) {
  duration_ = duration;
  estimate_stamp_ = Duration::zero();
  estimate_covariance_stamp_ = Duration::zero();
}

//-----------------------------------------------------------------------------
const double& R2WPFResults::get_x() const {
  lazy_compute_estimate_();
  return estimate_(R2WPFMetaState::POSITION_X);
}

//-----------------------------------------------------------------------------
const double& R2WPFResults::get_y() const {
  lazy_compute_estimate_();
  return estimate_(R2WPFMetaState::POSITION_Y);
}

//-----------------------------------------------------------------------------
const double& R2WPFResults::get_yaw() const {
  lazy_compute_estimate_();
  return estimate_(R2WPFMetaState::ORIENTATION_Z);
}

//-----------------------------------------------------------------------------
const double& R2WPFResults::get_yaw_variance() const {
  lazy_compute_estimate_();
  lazy_compute_estimate_covariance_();
  return estimate_(R2WPFMetaState::ORIENTATION_Z);
}

//-----------------------------------------------------------------------------
Eigen::Vector3d R2WPFResults::get_pose() const {
  lazy_compute_estimate_();
  return estimate_;
}

//-----------------------------------------------------------------------------
Eigen::Matrix3d R2WPFResults::get_pose_covariance() const {
  lazy_compute_estimate_();
  lazy_compute_estimate_covariance_();
  return estimate_covariance_;
}

//-----------------------------------------------------------------------------
const double& R2WPFResults::get_linear_speed() const {
  return input.U(LINEAR_SPEED_X_BODY);
}

//-----------------------------------------------------------------------------
const double& R2WPFResults::get_lateral_speed() const {
  return input.U(LINEAR_SPEED_Y_BODY);
}

//-----------------------------------------------------------------------------
const double& R2WPFResults::get_angular_speed() const {
  return input.U(ANGULAR_SPEED_Z_BODY);
}

//-----------------------------------------------------------------------------
Eigen::Vector3d R2WPFResults::get_twist() const { return input.U(); }

//-----------------------------------------------------------------------------
Eigen::Matrix3d R2WPFResults::get_twist_covariance() const {
  return input.QU();
}

//-----------------------------------------------------------------------------
void R2WPFResults::lazy_compute_estimate_() const {
  if (estimate_stamp_ != duration_) {
    weight_sum_ = state.weights.sum();
    this->estimate_(0) =
        (state.particles.row(0) * state.weights).sum() / weight_sum_;
    this->estimate_(1) =
        (state.particles.row(1) * state.weights).sum() / weight_sum_;
    double C =
        (state.particles.row(2).cos() * state.weights).sum() / weight_sum_;
    double S =
        (state.particles.row(2).sin() * state.weights).sum() / weight_sum_;
    this->estimate_(2) = std::atan2(S, C);
    estimate_stamp_ = duration_;
  }
}

//-----------------------------------------------------------------------------
void R2WPFResults::lazy_compute_estimate_covariance_() const {
  if (estimate_covariance_stamp_ != duration_) {
    for (int i = 0; i < STATE_SIZE; ++i) {
      this->mean_centered_particles_.row(i) =
          state.particles.row(i) - this->estimate_(i);
    }

    // TODO(jean) à vectoriser
    auto courseRow = this->mean_centered_particles_.row(2);
    for (int n = 0; n < state.particles.cols(); ++n) {
      courseRow(n) = betweenMinusPiAndPi(courseRow(n));
    }

    for (int i = 0; i < STATE_SIZE; ++i) {
      for (int j = i; j < STATE_SIZE; ++j) {
        this->estimate_covariance_(i, j) = this->estimate_covariance_(j, i) =
            (this->mean_centered_particles_.row(i) *
             this->mean_centered_particles_.row(j) * state.weights)
                .sum() /
            weight_sum_;
      }
    }

    estimate_covariance_stamp_ = duration_;
  }
}

//-----------------------------------------------------------------------------
Pose2D R2WPFResults::to_pose2d() const {
  lazy_compute_estimate_();
  lazy_compute_estimate_covariance_();

  Pose2D pose2D;
  pose2D.position.x() = estimate_(POSITION_X);
  pose2D.position.y() = estimate_(POSITION_Y);
  pose2D.yaw = estimate_(ORIENTATION_Z);
  pose2D.covariance = estimate_covariance_;
  return pose2D;
}

//-----------------------------------------------------------------------------
Pose3D R2WPFResults::to_pose3d() const {
  lazy_compute_estimate_();
  lazy_compute_estimate_covariance_();

  Pose3D pose3d;
  pose3d.position.x() = estimate_(POSITION_X);
  pose3d.position.y() = estimate_(POSITION_Y);
  pose3d.orientation.x() = addon.roll;
  pose3d.orientation.y() = addon.pitch;
  pose3d.orientation.z() = estimate_(ORIENTATION_Z);
  pose3d.covariance = toSe3Covariance(estimate_covariance_);
  pose3d.covariance(3, 3) = pose3d.covariance(4, 4) = addon.roll_pitch_variance;
  return pose3d;
}

//-----------------------------------------------------------------------------
PoseAndTwist2D R2WPFResults::to_pose_and_body_twist2d() const {
  lazy_compute_estimate_();
  lazy_compute_estimate_covariance_();

  PoseAndTwist2D poseAndTwist2D;
  poseAndTwist2D.pose.position.x() = estimate_(POSITION_X);
  poseAndTwist2D.pose.position.y() = estimate_(POSITION_Y);
  poseAndTwist2D.pose.yaw = estimate_(ORIENTATION_Z);
  poseAndTwist2D.pose.covariance = estimate_covariance_;
  poseAndTwist2D.twist.linearSpeeds.x() = get_linear_speed();
  poseAndTwist2D.twist.linearSpeeds.y() = get_lateral_speed();
  poseAndTwist2D.twist.angularSpeed = get_angular_speed();
  poseAndTwist2D.twist.covariance = get_twist_covariance();
  return poseAndTwist2D;
}

//-----------------------------------------------------------------------------
PoseAndTwist3D R2WPFResults::to_pose_and_body_twist3d() const {
  lazy_compute_estimate_();
  lazy_compute_estimate_covariance_();

  PoseAndTwist3D poseAndTwist3D;
  poseAndTwist3D.pose.position.x() = estimate_(POSITION_X);
  poseAndTwist3D.pose.position.y() = estimate_(POSITION_Y);
  poseAndTwist3D.pose.orientation.x() = addon.roll;
  poseAndTwist3D.pose.orientation.y() = addon.pitch;
  poseAndTwist3D.pose.orientation.z() = estimate_(ORIENTATION_Z);
  poseAndTwist3D.twist.linearSpeeds.x() = get_linear_speed();
  poseAndTwist3D.twist.linearSpeeds.y() = get_lateral_speed();
  poseAndTwist3D.twist.angularSpeeds.z() = get_angular_speed();
  poseAndTwist3D.pose.covariance = toSe3Covariance(estimate_covariance_);
  poseAndTwist3D.twist.covariance = toSe3Covariance(get_twist_covariance());
  poseAndTwist3D.pose.covariance(3, 3) = addon.roll_pitch_variance;
  poseAndTwist3D.pose.covariance(4, 4) = addon.roll_pitch_variance;
  return poseAndTwist3D;
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
