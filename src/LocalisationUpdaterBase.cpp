// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// std
#include <string>

// local
#include "romea_core_localisation/LocalisationUpdaterBase.hpp"

namespace romea
{
namespace core
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

}  // namespace core
}  // namespace romea
