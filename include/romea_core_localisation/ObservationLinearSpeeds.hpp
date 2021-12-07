#ifndef _romea_ObservationLinearSpeeds_HPP_
#define _romea_ObservationLinearSpeeds_HPP_

#include <romea_core_filtering/GaussianObservation.hpp>

namespace romea {


struct ObservationLinearSpeeds : GaussianObservation<double,2>
{
  enum Index
  {
    LINEAR_SPEED_X_BODY=0,
    LINEAR_SPEED_Y_BODY=1,
    SIZE
  };
};

}//romea

#endif
