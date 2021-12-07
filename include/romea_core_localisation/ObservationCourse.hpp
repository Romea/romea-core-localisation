#ifndef romea_ObservationCourse_hpp
#define romea_ObservationCourse_hpp

#include "romea_core_filtering/GaussianObservation.hpp"

namespace romea
{
 using ObservationCourse = GaussianObservation<double,1>;
}

#endif
