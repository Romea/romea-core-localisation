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
#include "romea_core_localisation/robot_to_world/particle/R2WLocalisationPFResults.hpp"
#include <romea_core_common/math/EulerAngles.hpp>

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
R2WLocalisationPFResults::R2WLocalisationPFResults(const size_t & numberOfParticles)
: R2WLocalisationResults<R2WLocalisationPFMetaState>(numberOfParticles),
  weightSum_(0),
  estimateStamp_(Duration::zero()),
  estimate_(Eigen::Vector3d::Zero()),
  estimateCovarianceStamp_(Duration::zero()),
  estimateCovariance_(Eigen::Matrix3d::Zero()),
  meanCenteredParticles_(RowMajorMatrix::Zero(STATE_SIZE, numberOfParticles))
{
}

//-----------------------------------------------------------------------------
R2WLocalisationPFResults::~R2WLocalisationPFResults()
{
}

//-----------------------------------------------------------------------------
void R2WLocalisationPFResults::reset(const Duration & duration)
{
  duration_ = duration;
  estimateStamp_ = Duration::zero();
  estimateCovarianceStamp_ = Duration::zero();
}

//-----------------------------------------------------------------------------
const double & R2WLocalisationPFResults::getX() const
{
  lazyComputeEstimate_();
  return estimate_(R2WLocalisationPFMetaState::POSITION_X);
}

//-----------------------------------------------------------------------------
const double & R2WLocalisationPFResults::getY() const
{
  lazyComputeEstimate_();
  return estimate_(R2WLocalisationPFMetaState::POSITION_Y);
}

//-----------------------------------------------------------------------------
const double & R2WLocalisationPFResults::getYaw() const
{
  lazyComputeEstimate_();
  return estimate_(R2WLocalisationPFMetaState::ORIENTATION_Z);
}

//-----------------------------------------------------------------------------
const double & R2WLocalisationPFResults::getYawVariance() const
{
  lazyComputeEstimate_();
  lazyComputeEstimateVovariance_();
  return estimate_(R2WLocalisationPFMetaState::ORIENTATION_Z);
}

//-----------------------------------------------------------------------------
Eigen::Vector3d R2WLocalisationPFResults::getPose() const
{
  lazyComputeEstimate_();
  return estimate_;
}

//-----------------------------------------------------------------------------
Eigen::Matrix3d R2WLocalisationPFResults::getPoseCovariance() const
{
  lazyComputeEstimate_();
  lazyComputeEstimateVovariance_();
  return estimateCovariance_;
}

//-----------------------------------------------------------------------------
const double & R2WLocalisationPFResults::getLinearSpeed() const
{
  return input.U(LINEAR_SPEED_X_BODY);
}

//-----------------------------------------------------------------------------
const double & R2WLocalisationPFResults::getLateralSpeed() const
{
  return input.U(LINEAR_SPEED_Y_BODY);
}

//-----------------------------------------------------------------------------
const double & R2WLocalisationPFResults::getAngularSpeed() const
{
  return input.U(ANGULAR_SPEED_Z_BODY);
}

//-----------------------------------------------------------------------------
Eigen::Vector3d R2WLocalisationPFResults::getTwist() const
{
  return input.U();
}

//-----------------------------------------------------------------------------
Eigen::Matrix3d R2WLocalisationPFResults::getTwistCovariance() const
{
  return input.QU();
}


//-----------------------------------------------------------------------------
void R2WLocalisationPFResults::lazyComputeEstimate_()const
{
  if (estimateStamp_ != duration_) {
    weightSum_ = state.weights.sum();
    this->estimate_(0) = (state.particles.row(0) * state.weights).sum() / weightSum_;
    this->estimate_(1) = (state.particles.row(1) * state.weights).sum() / weightSum_;
    double C = (state.particles.row(2).cos() * state.weights).sum() / weightSum_;
    double S = (state.particles.row(2).sin() * state.weights).sum() / weightSum_;
    this->estimate_(2) = std::atan2(S, C);
    estimateStamp_ = duration_;
  }
}


//-----------------------------------------------------------------------------
void R2WLocalisationPFResults::lazyComputeEstimateVovariance_() const
{
  if (estimateCovarianceStamp_ != duration_) {
    for (int i = 0; i < STATE_SIZE; ++i) {
      this->meanCenteredParticles_.row(i) = state.particles.row(i) - this->estimate_(i);
    }

    // TODO(jean) à vectoriser
    auto courseRow = this->meanCenteredParticles_.row(2);
    for (int n = 0; n < state.particles.cols(); ++n) {
      courseRow(n) = betweenMinusPiAndPi(courseRow(n));
    }


    for (int i = 0; i < STATE_SIZE; ++i) {
      for (int j = i; j < STATE_SIZE; ++j) {
        this->estimateCovariance_(i, j) =
          this->estimateCovariance_(j, i) = (this->meanCenteredParticles_.row(i) *
          this->meanCenteredParticles_.row(j) *
          state.weights).sum() / weightSum_;
      }
    }

    estimateCovarianceStamp_ = duration_;
  }
}

//-----------------------------------------------------------------------------
Pose2D R2WLocalisationPFResults::toPose2D() const
{
  lazyComputeEstimate_();
  lazyComputeEstimateVovariance_();

  Pose2D pose2D;
  pose2D.position.x() = estimate_(POSITION_X);
  pose2D.position.y() = estimate_(POSITION_Y);
  pose2D.yaw = estimate_(ORIENTATION_Z);
  pose2D.covariance = estimateCovariance_;
  return pose2D;
}

//-----------------------------------------------------------------------------
Pose3D R2WLocalisationPFResults::toPose3D() const
{
  lazyComputeEstimate_();
  lazyComputeEstimateVovariance_();

  Pose3D pose3d;
  pose3d.position.x() = estimate_(POSITION_X);
  pose3d.position.y() = estimate_(POSITION_Y);
  pose3d.orientation.x() = addon.roll;
  pose3d.orientation.y() = addon.pitch;
  pose3d.orientation.z() = estimate_(ORIENTATION_Z);
  pose3d.covariance = toSe3Covariance(estimateCovariance_);
  pose3d.covariance(3, 3) = pose3d.covariance(4, 4) = addon.rollPitchVariance;
  return pose3d;
}


//-----------------------------------------------------------------------------
PoseAndTwist2D R2WLocalisationPFResults::toPoseAndBodyTwist2D() const
{
  lazyComputeEstimate_();
  lazyComputeEstimateVovariance_();

  PoseAndTwist2D poseAndTwist2D;
  poseAndTwist2D.pose.position.x() = estimate_(POSITION_X);
  poseAndTwist2D.pose.position.y() = estimate_(POSITION_Y);
  poseAndTwist2D.pose.yaw = estimate_(ORIENTATION_Z);
  poseAndTwist2D.pose.covariance = estimateCovariance_;
  poseAndTwist2D.twist.linearSpeeds.x() = getLinearSpeed();
  poseAndTwist2D.twist.linearSpeeds.y() = getLateralSpeed();
  poseAndTwist2D.twist.angularSpeed = getAngularSpeed();
  poseAndTwist2D.twist.covariance = getTwistCovariance();
  return poseAndTwist2D;
}

//-----------------------------------------------------------------------------
PoseAndTwist3D R2WLocalisationPFResults::toPoseAndBodyTwist3D() const
{
  lazyComputeEstimate_();
  lazyComputeEstimateVovariance_();

  PoseAndTwist3D poseAndTwist3D;
  poseAndTwist3D.pose.position.x() = estimate_(POSITION_X);
  poseAndTwist3D.pose.position.y() = estimate_(POSITION_Y);
  poseAndTwist3D.pose.orientation.x() = addon.roll;
  poseAndTwist3D.pose.orientation.y() = addon.pitch;
  poseAndTwist3D.pose.orientation.z() = estimate_(ORIENTATION_Z);
  poseAndTwist3D.twist.linearSpeeds.x() = getLinearSpeed();
  poseAndTwist3D.twist.linearSpeeds.y() = getLateralSpeed();
  poseAndTwist3D.twist.angularSpeeds.z() = getAngularSpeed();
  poseAndTwist3D.pose.covariance = toSe3Covariance(estimateCovariance_);
  poseAndTwist3D.twist.covariance = toSe3Covariance(getTwistCovariance());
  poseAndTwist3D.pose.covariance(3, 3) = addon.rollPitchVariance;
  poseAndTwist3D.pose.covariance(4, 4) = addon.rollPitchVariance;
  return poseAndTwist3D;
}

}  // namespace core
}  // namespace romea
