#ifndef romea_LocalisationStoppingCriteria_hpp
#define romea_LocalisationStoppingCriteria_hpp

#include <romea_core_common/time/Time.hpp>

namespace romea
{

struct LocalisationStoppingCriteria
{

  LocalisationStoppingCriteria():
    maximalDurationInDeadReckoning(Duration::max()),
    maximalTravelledDistanceInDeadReckoning(std::numeric_limits<double>::max()),
    maximalPositionCircularErrorProbability(std::numeric_limits<double>::max())
  {

  }

  Duration maximalDurationInDeadReckoning;
  double maximalTravelledDistanceInDeadReckoning;
  double maximalPositionCircularErrorProbability;
};

}

#endif
