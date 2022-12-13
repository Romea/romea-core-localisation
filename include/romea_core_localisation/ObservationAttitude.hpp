#ifndef ROMEA_CORE_LOCALISATION_OBSERVATIONATTITUDE_HPP_
#define ROMEA_CORE_LOCALISATION_OBSERVATIONATTITUDE_HPP_

#include <romea_core_filtering/GaussianObservation.hpp>

namespace romea {

struct ObservationAttitude : GaussianObservation<double, 2>
{
  enum Index {
    ROLL = 0,
    PITCH,
    SIZE
  };
};

}  // namespace romea


#endif  // ROMEA_CORE_LOCALISATION_OBSERVATIONATTITUDE_HPP_
