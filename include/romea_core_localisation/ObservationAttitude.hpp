#ifndef romea_ObservationAttitude_hpp
#define romea_ObservationAttitude_hpp

#include <romea_core_filtering/GaussianObservation.hpp>

namespace romea {

struct ObservationAttitude : GaussianObservation<double,2>
{
  enum Index {
    ROLL = 0,
    PITCH,
    SIZE
  };
};

}


#endif
