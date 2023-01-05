// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATERBASE_HPP_
#define ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATERBASE_HPP_

// romea
#include <romea_core_common/diagnostic/CheckupRate.hpp>
#include <romea_core_common/diagnostic/DiagnosticReport.hpp>

// std
#include <mutex>
#include <string>

// local
#include "romea_core_localisation/LocalisationUpdaterTriggerMode.hpp"

namespace romea
{

class LocalisationUpdaterBase
{
public:
  using TriggerMode = LocalisationUpdaterTriggerMode;

public:
  LocalisationUpdaterBase(
    const std::string & updaterName,
    const double & minimalRate,
    const TriggerMode & triggerMode);

  bool heartBeatCallback(const Duration & duration);

  DiagnosticReport getReport();

protected:
  void udapteDiagnostic_(const Duration & duration);

protected:
  TriggerMode triggerMode_;
  CheckupGreaterThanRate rateDiagnostic_;
  mutable std::mutex mutex_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATERBASE_HPP_
