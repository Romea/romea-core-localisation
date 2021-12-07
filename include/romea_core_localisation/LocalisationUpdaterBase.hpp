#ifndef romea_LocalisationUpdaterBase_hpp
#define romea_LocalisationUpdaterBase_hpp

//romea
#include <romea_core_common/diagnostic/CheckupRate.hpp>
#include <romea_core_common/diagnostic/DiagnosticReport.hpp>
#include "LocalisationUpdaterTriggerMode.hpp"


//std
#include <mutex>

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
  CheckupRate rateDiagnostic_;
  mutable std::mutex mutex_;
};

}//romea

#endif
