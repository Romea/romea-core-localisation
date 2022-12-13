#ifndef ROMEA_CORE_LOCALISATION_OBSERVATIONCOURSE_HPP_
#define ROMEA_CORE_LOCALISATION_OBSERVATIONCOURSE_HPP_

#include "romea_core_filtering/GaussianObservation.hpp"

namespace romea
{
 using ObservationCourse = GaussianObservation<double, 1>;
}

#endif  // ROMEA_CORE_LOCALISATION_OBSERVATIONCOURSE_HPP_
