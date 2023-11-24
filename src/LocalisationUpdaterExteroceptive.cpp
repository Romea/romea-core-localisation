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
#include <vector>

// local
#include "romea_core_localisation/LocalisationUpdaterExteroceptive.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
LocalisationUpdaterExteroceptive::LocalisationUpdaterExteroceptive(
  const std::string & updaterName,
  const double & minimalRate,
  const TriggerMode & triggerMode,
  const std::string & logFilename)
: LocalisationUpdaterBase(updaterName,
    minimalRate,
    triggerMode)
{
  openLogFile_(logFilename);
}


//-----------------------------------------------------------------------------
void LocalisationUpdaterExteroceptive::openLogFile_(const std::string & logFilename)
{
  if (!logFilename.empty()) {
    logFile_.open(logFilename);

    if (!logFile_.is_open()) {
      throw std::runtime_error("Cannot open debug file : " + logFilename);
    }
  }
}

//-----------------------------------------------------------------------------
void LocalisationUpdaterExteroceptive::setLogFileHeader_(
  const std::vector<std::string> & logColumnNames)
{
  if (logFile_.is_open()) {
    logFile_ << "%";
    for (size_t n = 0; n < logColumnNames.size(); ++n) {
      logFile_ << "(" << n + 1 << ")" << logColumnNames[n] << ",";
    }
    logFile_ << "\n";
  }
}

}  // namespace core
}  // namespace romea
