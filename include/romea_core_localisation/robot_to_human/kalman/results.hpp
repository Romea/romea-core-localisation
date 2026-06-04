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

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__RESULTS_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__RESULTS_HPP_

#include "romea_core_common/geometry/Position2D.hpp"
#include "romea_core_localisation/robot_to_human/kalman/meta_state.hpp"
#include "romea_core_localisation/robot_to_human/results_base.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

class R2HKFResults : public R2HResultsBase<R2HKFMetaState>
{
public:
  R2HKFResults();
  virtual ~R2HKFResults() = default;

  const double & get_leader_x() const override;
  const double & get_leader_y() const override;

  Eigen::Vector2d get_leader_position() const override;
  Eigen::Matrix2d get_leader_position_covariance() const override;

  const double & get_linear_speed() const override;
  const double & get_lateral_speed() const override;
  const double & get_angular_speed() const override;

  Eigen::Vector3d get_twist() const override;
  Eigen::Matrix3d get_twist_covariance() const override;

  Position2D to_leader_position2d() const override;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_HUMAN__KALMAN__RESULTS_HPP_
