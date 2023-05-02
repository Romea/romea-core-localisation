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


#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__R2HLOCALISATIONKFRESULTS_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__R2HLOCALISATIONKFRESULTS_HPP_

#include <romea_core_common/geometry/Position2D.hpp>
#include "romea_core_localisation/robot_to_human/R2HLocalisationResults.hpp"
#include "romea_core_localisation/robot_to_human/kalman/R2HLocalisationKFMetaState.hpp"

namespace romea
{

class R2HLocalisationKFResults : public R2HLocalisationResults<R2HLocalisationKFMetaState>
{
public:
  R2HLocalisationKFResults();
  virtual ~R2HLocalisationKFResults() = default;

  const double & getLeaderX() const override;
  const double & getLeaderY() const override;

  Eigen::Vector2d getLeaderPosition() const override;
  Eigen::Matrix2d getLeaderPositionCovariance() const override;

  const double & getLinearSpeed() const override;
  const double & getLateralSpeed() const override;
  const double & getAngularSpeed() const override;

  Eigen::Vector3d getTwist() const override;
  Eigen::Matrix3d getTwistCovariance() const override;

  Position2D toLeaderPosition2D() const override;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__R2HLOCALISATIONKFRESULTS_HPP_
