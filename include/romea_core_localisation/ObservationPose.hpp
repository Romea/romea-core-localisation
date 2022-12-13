#ifndef ROMEA_CORE_LOCALISATION_OBSERVATIONPOSE_HPP_
#define ROMEA_CORE_LOCALISATION_OBSERVATIONPOSE_HPP_


#include <romea_core_filtering/GaussianObservation.hpp>

namespace romea
{
struct ObservationPose : GaussianObservation<double, 3>
{
  enum Index {
    POSITION_X = 0,
    POSITION_Y,
    ORIENTATION_Z,
    SIZE
  };


  ObservationPose():
    GaussianObservation(),
    levelArm(Eigen::Vector3d::Zero())
  {
  }

  Eigen::Vector3d levelArm;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_OBSERVATIONPOSE_HPP_
