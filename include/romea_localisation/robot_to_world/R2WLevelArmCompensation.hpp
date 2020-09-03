#ifndef romea_LevelArmCompensation_hpp
#define romea_LevelArmCompensation_hpp

//romea
#include <romea_common/transform/SmartRotation3D.hpp>

namespace romea {

class LevelArmCompensation
{

public :

  LevelArmCompensation();

  void compute(const double & vehicleRollAngle,
               const double & vehiclePitchAngle,
               const double & vehicleRollPitchVariance,
               const double & vehicleYawAngle,
               const double & vehicleYawAngleVariance,
               const Eigen::Vector3d & bodyAntennaPosition);

  const Eigen::Vector3d & getPosition()const;
  const Eigen::Matrix3d & getPositionCovariance()const;
  const Eigen::Matrix3d & getJacobian()const;

private :

  Eigen::Vector3d position_;
  Eigen::Matrix3d positionCovariance_;

  SmartRotation3D vehicleAttitude_;
  Eigen::Matrix3d vehicleAttitudeCovariance_;
  Eigen::Matrix3d jacobian_;

};

}

#endif
