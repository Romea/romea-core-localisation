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


#include "romea_core_localisation/robot_to_world/R2WLevelArmCompensation.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
LevelArmCompensation::LevelArmCompensation()
: position_(Eigen::Vector3d::Zero()),
  positionCovariance_(Eigen::Matrix3d::Zero()),
  vehicleAttitude_(),
  vehicleAttitudeCovariance_(Eigen::Matrix3d::Zero()),
  jacobian_(Eigen::Matrix3d::Zero())
{
}

//-----------------------------------------------------------------------------
void LevelArmCompensation::compute(
  const double & vehicleRollAngle,
  const double & vehiclePitchAngle,
  const double & vehicleRollPitchVariance,
  const double & vehicleYawAngle,
  const double & vehicleYawAngleVariance,
  const Eigen::Vector3d & bodyAntennaPosition)
{
  // Compute full vehicle atiitude
  vehicleAttitudeCovariance_(0, 0) = vehicleRollPitchVariance;
  vehicleAttitudeCovariance_(1, 1) = vehicleRollPitchVariance;
  vehicleAttitudeCovariance_(2, 2) = vehicleYawAngleVariance;

  vehicleAttitude_.init(vehicleRollAngle, vehiclePitchAngle, vehicleYawAngle);

  // Compute antenna position and its covariance
  position_ = vehicleAttitude_ * bodyAntennaPosition;
  jacobian_ = vehicleAttitude_.dRTdAngles(bodyAntennaPosition);
  positionCovariance_ = jacobian_ * vehicleAttitudeCovariance_ * jacobian_.transpose();
}

//-----------------------------------------------------------------------------
const Eigen::Vector3d & LevelArmCompensation::getPosition()const
{
  return position_;
}

//-----------------------------------------------------------------------------
const Eigen::Matrix3d & LevelArmCompensation::getPositionCovariance()const
{
  return positionCovariance_;
}

//-----------------------------------------------------------------------------
const Eigen::Matrix3d & LevelArmCompensation::getJacobian()const
{
  return jacobian_;
}

}  // namespace core
}  // namespace romea
