#ifndef ROMEA_CORE_LOCALISATION_OBSERVATIONANGULARSPEED_HPP_
#define ROMEA_CORE_LOCALISATION_OBSERVATIONANGULARSPEED_HPP_

#include <romea_core_filtering/GaussianObservation.hpp>

namespace romea
{
 using ObservationAngularSpeed = GaussianObservation<double, 1>;
}

#endif // ROMEA_CORE_LOCALISATION_OBSERVATIONANGULARSPEED_HPP_
