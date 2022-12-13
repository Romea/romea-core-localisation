#ifndef ROMEA_CORE_LOCALISATION_OBSERVATIONLINEARSPEED_HPP_
#define ROMEA_CORE_LOCALISATION_OBSERVATIONLINEARSPEED_HPP_

#include <romea_core_filtering/GaussianObservation.hpp>

namespace romea {
  using  ObservationLinearSpeed = GaussianObservation<double, 1>;
}

#endif  // ROMEA_CORE_LOCALISATION_OBSERVATIONLINEARSPEED_HPP_
