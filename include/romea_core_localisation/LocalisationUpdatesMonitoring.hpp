#ifndef ROMEA_CORE_LOCALISATION_LOCALISATIONUPDATESMONITORING_HPP_
#define ROMEA_CORE_LOCALISATION_LOCALISATIONUPDATESMONITORING_HPP_

// romea
#include <romea_core_common/time/Time.hpp>

namespace romea {


struct LocalisationUpdateMonitoring
{
public :

  LocalisationUpdateMonitoring();

  Duration time;
  double travelledDistance;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_LOCALISATIONUPDATESMONITORING_HPP_
