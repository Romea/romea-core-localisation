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


//std
#include <string>

//local
#include "romea_core_localisation/LocalisationFSMState.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
std::string toString(const LocalisationFSMState & fmsState)
{
  switch (fmsState) {
    case LocalisationFSMState::INIT:
      return "INIT";
    case LocalisationFSMState::RUNNING:
      return "RUNNING";
    case LocalisationFSMState::RESET:
      return "RESET";
    case LocalisationFSMState::ABORTED:
      return "ABORTED";
    default:
      return "";
  }
}

//-----------------------------------------------------------------------------
DiagnosticStatus toDiagnosticStatus(const LocalisationFSMState & fmsState)
{
  switch (fmsState) {
    case LocalisationFSMState::INIT:
      return DiagnosticStatus::WARN;
    case LocalisationFSMState::RUNNING:
      return DiagnosticStatus::OK;
    case LocalisationFSMState::RESET:
      return DiagnosticStatus::WARN;
    case LocalisationFSMState::ABORTED:
      return DiagnosticStatus::ERROR;
    default:
      return DiagnosticStatus::STALE;
  }
}

}  // namespace romea
