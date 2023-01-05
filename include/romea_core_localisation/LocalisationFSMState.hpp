// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION__LOCALISATIONFSMSTATE_HPP_
#define ROMEA_CORE_LOCALISATION__LOCALISATIONFSMSTATE_HPP_

// romea
#include <romea_core_common/diagnostic/DiagnosticStatus.hpp>

// std
#include <string>

namespace romea
{

enum class LocalisationFSMState
{
  INIT = 0,
  RUNNING,
  RESET,
  ABORTED
};

std::string toString(const LocalisationFSMState & fmsState);

DiagnosticStatus toDiagnosticStatus(const LocalisationFSMState & fmsState);

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__LOCALISATIONFSMSTATE_HPP_
