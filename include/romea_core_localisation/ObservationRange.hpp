#ifndef ROMEA_CORE_LOCALISATION_OBSERVATIONRANGE_HPP_
#define ROMEA_CORE_LOCALISATION_OBSERVATIONRANGE_HPP_

#include <romea_core_filtering/GaussianObservation.hpp>

namespace romea {

struct ObservationRange : GaussianObservation<double, 1>
{
  ObservationRange():
    responderPosition(Eigen::Vector3d::Zero()),
    initiatorPosition(Eigen::Vector3d::Zero()),
    terrainElevation(0)
  {
  }

  Eigen::Vector3d responderPosition;
  Eigen::Vector3d initiatorPosition;
  double terrainElevation;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_OBSERVATIONRANGE_HPP_
