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


#ifndef ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATEREXTEROCEPTIVE_HPP_
#define ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATEREXTEROCEPTIVE_HPP_

// std
#include <fstream>
#include <vector>
#include <string>

// romea
#include "romea_core_localisation/LocalisationUpdaterBase.hpp"

namespace romea
{

class LocalisationUpdaterExteroceptive : public LocalisationUpdaterBase
{
public:
  LocalisationUpdaterExteroceptive(
    const std::string & updaterName,
    const double & minimalRate,
    const TriggerMode & triggerMode,
    const std::string & logFilename);

  virtual ~LocalisationUpdaterExteroceptive() = default;

  void openLogFile_(const std::string & logFilename);

  void setLogFileHeader_(const std::vector<std::string> & logColumnNames);

protected:
  std::ofstream logFile_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__LOCALISATIONUPDATEREXTEROCEPTIVE_HPP_
