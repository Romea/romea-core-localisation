#ifndef romea_UpdatesMonitoring_hpp
#define romea_UpdatesMonitoring_hpp

// romea
#include <romea_common/time/Time.hpp>

namespace romea {


struct LocalisationUpdateMonitoring
{

public :

  LocalisationUpdateMonitoring();

  Duration time;
  double travelledDistance;

};

}


#endif
