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

#include "romea_core_localisation/robot_to_human/kalman/results.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
R2HKFResults::R2HKFResults() : R2HResultsBase()
{
}

//--------------------------------------------------------------------------
const double & R2HKFResults::get_leader_x() const
{
  return state.X(R2HKFMetaState::LEADER_POSITION_X);
}

//--------------------------------------------------------------------------
const double & R2HKFResults::get_leader_y() const
{
  return state.X(R2HKFMetaState::LEADER_POSITION_Y);
}

//--------------------------------------------------------------------------
const double & R2HKFResults::get_linear_speed() const
{
  return input.U(R2HKFMetaState::LINEAR_SPEED_X_BODY);
}

//--------------------------------------------------------------------------
const double & R2HKFResults::get_lateral_speed() const
{
  return input.U(R2HKFMetaState::LINEAR_SPEED_Y_BODY);
}

//--------------------------------------------------------------------------
const double & R2HKFResults::get_angular_speed() const
{
  return input.U(R2HKFMetaState::ANGULAR_SPEED_Z_BODY);
}

//--------------------------------------------------------------------------
Eigen::Vector3d R2HKFResults::get_twist() const
{
  return input.U();
}

//--------------------------------------------------------------------------
Eigen::Matrix3d R2HKFResults::get_twist_covariance() const
{
  return input.QU();
}

//--------------------------------------------------------------------------
Eigen::Vector2d R2HKFResults::get_leader_position() const
{
  return state.X();
}

//--------------------------------------------------------------------------
Eigen::Matrix2d R2HKFResults::get_leader_position_covariance() const
{
  return state.P();
}

//-----------------------------------------------------------------------------
Position2D R2HKFResults::to_leader_position2d() const
{
  Position2D position2d;
  position2d.position.x() = get_leader_x();
  position2d.position.y() = get_leader_y();
  position2d.covariance = get_leader_position_covariance();
  return position2d;
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
