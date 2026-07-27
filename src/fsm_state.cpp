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

//  local
#include "romea_core_localisation/fsm_state.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
std::string to_string(const FSMState & fms_state)
{
  switch (fms_state) {
    case FSMState::INIT:
      return "INIT";
    case FSMState::RUNNING:
      return "RUNNING";
    case FSMState::RESET:
      return "RESET";
    case FSMState::ABORTED:
      return "ABORTED";
    case FSMState::STALE:
      return "STALE";
    default:
      return "";
  }
}

//-----------------------------------------------------------------------------
romea::core::FSMState to_common_fsm_state(const FSMState & fsm_state)
{
  return {to_string(fsm_state), static_cast<uint8_t>(fsm_state)};
}

//-----------------------------------------------------------------------------
romea::core::FSMEvent make_fsm_event(
  const FSMState & previous_state,
  const FSMState & current_state,
  const std::string & description)
{
  return {to_common_fsm_state(previous_state), to_common_fsm_state(current_state), description};
}

//-----------------------------------------------------------------------------
DiagnosticStatus to_diagnostic_status(const FSMState & fms_state)
{
  switch (fms_state) {
    case FSMState::INIT:
      return DiagnosticStatus::WARN;
    case FSMState::RUNNING:
      return DiagnosticStatus::OK;
    case FSMState::RESET:
      return DiagnosticStatus::WARN;
    case FSMState::ABORTED:
      return DiagnosticStatus::ERROR;
    case FSMState::STALE:
      return DiagnosticStatus::STALE;
    default:
      return DiagnosticStatus::STALE;
  }
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
