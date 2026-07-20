// Copyright 2022 INRAE, French National Research Institute for Agriculture,
// Food and Environment
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

#ifndef ROMEA_CORE_LOCALISATION__UPDATER_BASE_HPP_
#define ROMEA_CORE_LOCALISATION__UPDATER_BASE_HPP_

// romea
#include <romea_core_common/diagnostic/CheckupRate.hpp>
#include <romea_core_common/diagnostic/DiagnosticReport.hpp>
#include <romea_core_common/fsm/FSMEventNotifier.hpp>

// std
#include <mutex>
#include <string>

// local
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/updater_trigger_mode.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

class UpdaterBase
{
public:
  using trigger_mode = Updatertrigger_mode;

public:
  UpdaterBase(
    const std::string & updater_name,
    const double & minimal_rate,
    const trigger_mode & trigger_mode);

  bool heart_beat_callback(const Duration & duration);

  void register_fsm_event_callback(FSMEventCallback callback);

  DiagnosticReport get_report();

protected:
  void udapte_diagnostic_(const Duration & duration);

  void notify_fsm_event_(
    const FSMState & previous_state,
    const FSMState & current_state,
    const std::string & description);

protected:
  trigger_mode trigger_mode_;
  CheckupGreaterThanRate rate_diagnostic_;
  FSMEventNotifier fsm_event_notifier_;
  mutable std::mutex mutex_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__UPDATER_BASE_HPP_
