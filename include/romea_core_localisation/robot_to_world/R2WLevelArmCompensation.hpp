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
