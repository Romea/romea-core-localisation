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


#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__R2WLOCALISATIONPFRESULTS_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__R2WLOCALISATIONPFRESULTS_HPP_

// romea
#include <romea_core_filtering/particle/ParticleFilterEstimator.hpp>
#include "romea_core_localisation/robot_to_world/R2WLocalisationResults.hpp"
#include "romea_core_localisation/robot_to_world/particle/R2WLocalisationPFMetaState.hpp"

namespace romea
{
namespace core
{

class R2WLocalisationPFResults : public R2WLocalisationResults<R2WLocalisationPFMetaState>
{
public:
  using  RowMajorMatrix = R2WLocalisationPFMetaState::State::RowMajorMatrix;

public:
  explicit R2WLocalisationPFResults(const size_t & numberOfParticles);
  virtual ~R2WLocalisationPFResults();

  const double & getX() const override;
  const double & getY() const override;
  const double & getYaw() const override;
  const double & getYawVariance() const override;

  Eigen::Vector3d getPose() const override;
  Eigen::Matrix3d getPoseCovariance() const override;

  const double & getLinearSpeed() const override;
  const double & getLateralSpeed() const override;
  const double & getAngularSpeed() const override;

  Eigen::Vector3d getTwist() const override;
  Eigen::Matrix3d getTwistCovariance() const override;

  Pose2D toPose2D() const override;
  Pose3D toPose3D() const override;

  PoseAndTwist2D toPoseAndBodyTwist2D() const override;
  PoseAndTwist3D toPoseAndBodyTwist3D() const override;

  void reset(const Duration & duration);

private:
  void lazyComputeEstimate_() const;
  void lazyComputeEstimateVovariance_() const;

private:
  mutable double weightSum_;

  mutable Duration estimateStamp_;
  mutable Eigen::Vector3d estimate_;

  mutable Duration estimateCovarianceStamp_;
  mutable Eigen::Matrix3d estimateCovariance_;

  mutable RowMajorMatrix meanCenteredParticles_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__R2WLOCALISATIONPFRESULTS_HPP_
