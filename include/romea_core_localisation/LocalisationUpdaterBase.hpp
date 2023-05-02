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
