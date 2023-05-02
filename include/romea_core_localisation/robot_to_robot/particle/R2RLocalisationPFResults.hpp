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


#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__PARTICLE__R2RLOCALISATIONPFRESULTS_HPP
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__PARTICLE__R2RLOCALISATIONPFRESULTS_HPP

#include "romea_core_localisation/robot_to_robot/R2RLocalisationResults.hpp"
#include "romea_core_localisation/robot_to_robot/particle/R2RLocalisationPFMetaState.hpp"

namespace romea
{

class R2RLocalisationPFResults : public R2RLocalisationResults<R2RLocalisationPFMetaState>
{
public:
  explicit R2RLocalisationPFResults(const size_t & numberOfParticles);
  virtual ~R2RLocalisationPFResults() = default;

  const double & getLeaderX() const override;
  const double & getLeaderY() const override;
  const double & getLeaderOrientation() const override;

  Eigen::Vector3d getLeaderPose() const override;
  Eigen::Matrix3d getLeaderPoseCovariance() const override;

  const double & getLinearSpeed() const override;
  const double & getLateralSpeed() const override;
  const double & getAngularSpeed() const override;

  Eigen::Vector3d getTwist() const override;
  Eigen::Matrix3d getTwistCovariance() const override;

  const double & getLeaderLinearSpeed() const override;
  const double & getLeaderLateralSpeed() const override;
  const double & getLeaderAngularSpeed() const override;

  Eigen::Vector3d getLeaderTwist() const override;
  Eigen::Matrix3d getLeaderTwistCovariance() const override;

  Pose2D toLeaderPose2D() const override;
  PoseAndTwist2D toLeaderPoseAndBodyTwist2D() const override;

private:
  void lazyComputeEstimate_() const;
  void lazyComputeEstimateVovariance_() const;

private:
  mutable double weightSum_;

  mutable Duration estimateStamp_;
  mutable Eigen::Vector3d estimate_;

  mutable Duration estimateCovarianceStamp_;
  mutable Eigen::Matrix3d estimateCovariance_;

  mutable R2RLocalisationPFMetaState::State::RowMajorMatrix meanCenteredParticles_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__PARTICLE__R2RLOCALISATIONPFRESULTS_HPP
