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

#ifndef ROMEA_CORE_LOCALISATION__FSM_STATE_HPP_
#define ROMEA_CORE_LOCALISATION__FSM_STATE_HPP_

// romea
#include <romea_core_common/diagnostic/DiagnosticStatus.hpp>
#include <romea_core_common/fsm/FSMEvent.hpp>
#include <romea_core_common/fsm/FSMState.hpp>

// std
#include <string>

namespace romea
{

namespace core
{

namespace localisation
{

enum class FSMState
{
  INIT = 0,
  RUNNING,
  RESET,
  ABORTED
};

std::string to_string(const FSMState & fms_state);

romea::core::FSMState to_common_fsm_state(const FSMState & fsm_state);

romea::core::FSMEvent make_fsm_event(
  const FSMState & previous_state,
  const FSMState & current_state,
  const std::string & description);

DiagnosticStatus to_diagnostic_status(const FSMState & fms_state);

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__FSM_STATE_HPP_
