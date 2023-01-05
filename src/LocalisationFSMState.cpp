// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

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
