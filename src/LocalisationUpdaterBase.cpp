// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <string>

// local
#include "romea_core_localisation/LocalisationUpdaterBase.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
LocalisationUpdaterBase::LocalisationUpdaterBase(
  const std::string & updaterName,
  const double & minimalRate,
  const TriggerMode & triggerMode)
: triggerMode_(triggerMode),
  rateDiagnostic_(updaterName, minimalRate, 0.1 * minimalRate),
  mutex_()
{
}

//-----------------------------------------------------------------------------
bool LocalisationUpdaterBase::heartBeatCallback(const Duration & duration)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return rateDiagnostic_.heartBeatCallback(duration);
}

//-----------------------------------------------------------------------------
DiagnosticReport LocalisationUpdaterBase::getReport()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return rateDiagnostic_.getReport();
}


//-----------------------------------------------------------------------------
void LocalisationUpdaterBase::udapteDiagnostic_(const Duration & duration)
{
  std::lock_guard<std::mutex> lock(mutex_);
  rateDiagnostic_.evaluate(duration);
}

}  // namespace romea
