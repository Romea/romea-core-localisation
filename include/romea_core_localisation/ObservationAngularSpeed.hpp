// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__OBSERVATIONANGULARSPEED_HPP_
#define ROMEA_CORE_LOCALISATION_OBSERVATIONANGULARSPEED_HPP_

#include <romea_core_filtering/GaussianObservation.hpp>

namespace romea
{
using ObservationAngularSpeed = GaussianObservation<double, 1>;
}

#endif // ROMEA_CORE_LOCALISATION__OBSERVATIONANGULARSPEED_HPP_
