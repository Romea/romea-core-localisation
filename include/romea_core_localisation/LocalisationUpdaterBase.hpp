#ifndef ROMEA_CORE_LOCALISATION_LOCALISATIONUPDATERBASE_HPP_
#define ROMEA_CORE_LOCALISATION_LOCALISATIONUPDATERBASE_HPP_

// std
#include <mutex>
#include <string>

// romea
#include <romea_core_common/diagnostic/CheckupRate.hpp>
#include <romea_core_common/diagnostic/DiagnosticReport.hpp>
#include "romea_core_localisation/LocalisationUpdaterTriggerMode.hpp"

namespace romea {

class LocalisationUpdaterBase
{
public :

  using TriggerMode =  LocalisationUpdaterTriggerMode;

public :

  LocalisationUpdaterBase(const std::string & updaterName,
                          const double & minimalRate,
                          const TriggerMode & triggerMode);

  bool heartBeatCallback(const Duration & duration);

  DiagnosticReport getReport();

protected :

  void udapteDiagnostic_(const Duration & duration);

protected :

  TriggerMode triggerMode_;
  CheckupGreaterThanRate rateDiagnostic_;
  mutable std::mutex mutex_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_LOCALISATIONUPDATERBASE_HPP_
