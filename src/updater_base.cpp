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
#include "romea_core_localisation/updater_base.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
UpdaterBase::UpdaterBase(
  const std::string & updater_name,
  const double & minimal_rate,
  const TriggerMode & trigger_mode)
: trigger_mode_(trigger_mode),
  rate_diagnostic_(updater_name, minimal_rate, 0.1 * minimal_rate),
  mutex_()
{
}

//-----------------------------------------------------------------------------
bool UpdaterBase::heart_beat_callback(const Duration & duration)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return rate_diagnostic_.heartBeatCallback(duration);
}

//-----------------------------------------------------------------------------
DiagnosticReport UpdaterBase::get_report()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return rate_diagnostic_.getReport();
}


//-----------------------------------------------------------------------------
void UpdaterBase::udapte_diagnostic_(const Duration & duration)
{
  std::lock_guard<std::mutex> lock(mutex_);
  rate_diagnostic_.evaluate(duration);
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
