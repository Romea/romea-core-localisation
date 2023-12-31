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


#include "romea_core_localisation/robot_to_human/kalman/R2HLocalisationKFResults.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
R2HLocalisationKFResults::R2HLocalisationKFResults()
: R2HLocalisationResults()
{
}

//--------------------------------------------------------------------------
const double & R2HLocalisationKFResults::getLeaderX() const
{
  return state.X(R2HLocalisationKFMetaState::LEADER_POSITION_X);
}

//--------------------------------------------------------------------------
const double & R2HLocalisationKFResults::getLeaderY() const
{
  return state.X(R2HLocalisationKFMetaState::LEADER_POSITION_Y);
}


//--------------------------------------------------------------------------
const double & R2HLocalisationKFResults::getLinearSpeed() const
{
  return input.U(R2HLocalisationKFMetaState::LINEAR_SPEED_X_BODY);
}

//--------------------------------------------------------------------------
const double & R2HLocalisationKFResults::getLateralSpeed() const
{
  return input.U(R2HLocalisationKFMetaState::LINEAR_SPEED_Y_BODY);
}

//--------------------------------------------------------------------------
const double & R2HLocalisationKFResults::getAngularSpeed() const
{
  return input.U(R2HLocalisationKFMetaState::ANGULAR_SPEED_Z_BODY);
}

//--------------------------------------------------------------------------
Eigen::Vector3d R2HLocalisationKFResults::getTwist() const
{
  return input.U();
}

//--------------------------------------------------------------------------
Eigen::Matrix3d R2HLocalisationKFResults::getTwistCovariance()const
{
  return input.QU();
}

//--------------------------------------------------------------------------
Eigen::Vector2d R2HLocalisationKFResults::getLeaderPosition() const
{
  return state.X();
}

//--------------------------------------------------------------------------
Eigen::Matrix2d R2HLocalisationKFResults::getLeaderPositionCovariance() const
{
  return state.P();
}

//-----------------------------------------------------------------------------
Position2D R2HLocalisationKFResults::toLeaderPosition2D() const
{
  Position2D position2d;
  position2d.position.x() = getLeaderX();
  position2d.position.y() = getLeaderY();
  position2d.covariance = getLeaderPositionCovariance();
  return position2d;
}

}  // namespace core
}  // namespace romea
