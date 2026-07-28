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

// std
#include <stdexcept>
#include <string>
#include <utility>

// local
#include "romea_core_localisation/updater_base.hpp"

namespace
{

//-----------------------------------------------------------------------------
double validate_minimal_rate(const std::string & updater_name, const double & minimal_rate)
{
  if (minimal_rate <= 0.0) {
    throw std::invalid_argument("Invalid minimal rate for updater " + updater_name);
  }

  return minimal_rate;
}

}  // namespace

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
UpdaterBase::UpdaterBase(
  const std::string & updater_name, const double & minimal_rate, const trigger_mode & trigger_mode)
: trigger_mode_(trigger_mode),
  minimal_rate_(validate_minimal_rate(updater_name, minimal_rate)),
  rate_diagnostic_(updater_name, minimal_rate_, 0.1 * minimal_rate_),
  fsm_event_notifier_(),
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
const double & UpdaterBase::get_minimal_rate() const
{
  return minimal_rate_;
}

//-----------------------------------------------------------------------------
void UpdaterBase::register_fsm_event_callback(FSMEventCallback callback)
{
  fsm_event_notifier_.register_callback(std::move(callback));
}

//-----------------------------------------------------------------------------
void UpdaterBase::udapte_diagnostic_(const Duration & duration)
{
  std::lock_guard<std::mutex> lock(mutex_);
  rate_diagnostic_.evaluate(duration);
}

//-----------------------------------------------------------------------------
void UpdaterBase::notify_fsm_event_(
  const FSMState & previous_state,
  const FSMState & current_state,
  const std::string & description)
{
  if (previous_state != current_state) {
    fsm_event_notifier_.notify(make_fsm_event(previous_state, current_state, description));
  }
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
