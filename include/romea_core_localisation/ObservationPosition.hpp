#ifndef ROMEA_CORE_LOCALISATION_OBSERVATIONPOSITION_HPP_
#define ROMEA_CORE_LOCALISATION_OBSERVATIONPOSITION_HPP_

// romea
#include <romea_core_filtering/GaussianObservation.hpp>

namespace romea {

struct ObservationPosition : GaussianObservation<double, 2>
{
  enum Index {
    POSITION_X = 0,
    POSITION_Y,
    SIZE
  };

  ObservationPosition():
    GaussianObservation(),
    levelArm(Eigen::Vector3d::Zero())
  {
  }

  Eigen::Vector3d levelArm;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_OBSERVATIONPOSITION_HPP_
