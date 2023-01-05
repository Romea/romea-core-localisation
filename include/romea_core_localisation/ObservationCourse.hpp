// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__OBSERVATIONCOURSE_HPP_
#define ROMEA_CORE_LOCALISATION__OBSERVATIONCOURSE_HPP_

#include "romea_core_filtering/GaussianObservation.hpp"

namespace romea
{
using ObservationCourse = GaussianObservation<double, 1>;
}

#endif  // ROMEA_CORE_LOCALISATION__OBSERVATIONCOURSE_HPP_
