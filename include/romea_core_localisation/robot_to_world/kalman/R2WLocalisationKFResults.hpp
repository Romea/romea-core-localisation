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


#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__R2WLOCALISATIONKFRESULTS_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__R2WLOCALISATIONKFRESULTS_HPP_

#include "romea_core_localisation/robot_to_world/R2WLocalisationResults.hpp"
#include "romea_core_localisation/robot_to_world/kalman/R2WLocalisationKFMetaState.hpp"

namespace romea
{
namespace core
{


class R2WLocalisationKFResults : public R2WLocalisationResults<R2WLocalisationKFMetaState>
{
public:
  R2WLocalisationKFResults();

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
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__KALMAN__R2WLOCALISATIONKFRESULTS_HPP_
