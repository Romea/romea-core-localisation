// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "romea_core_localisation/robot_to_world/R2WLevelArmCompensation.hpp"

namespace romea
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

}  // namespace romea
