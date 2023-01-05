// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__R2WLEVELARMCOMPENSATION_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__R2WLEVELARMCOMPENSATION_HPP_

// romea
#include <romea_core_common/transform/SmartRotation3D.hpp>

namespace romea
{

class LevelArmCompensation
{
public:
  LevelArmCompensation();

  void compute(
    const double & vehicleRollAngle,
    const double & vehiclePitchAngle,
    const double & vehicleRollPitchVariance,
    const double & vehicleYawAngle,
    const double & vehicleYawAngleVariance,
    const Eigen::Vector3d & bodyAntennaPosition);

  const Eigen::Vector3d & getPosition()const;
  const Eigen::Matrix3d & getPositionCovariance()const;
  const Eigen::Matrix3d & getJacobian()const;

private:
  Eigen::Vector3d position_;
  Eigen::Matrix3d positionCovariance_;

  SmartRotation3D vehicleAttitude_;
  Eigen::Matrix3d vehicleAttitudeCovariance_;
  Eigen::Matrix3d jacobian_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__R2WLEVELARMCOMPENSATION_HPP_
