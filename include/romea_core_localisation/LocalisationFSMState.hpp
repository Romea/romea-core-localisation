#ifndef ROMEA_CORE_LOCALISATION_LOCALISATIONFSMSTATE_HPP_
#define ROMEA_CORE_LOCALISATION_LOCALISATIONFSMSTATE_HPP_

#include <string>
#include <romea_core_common/diagnostic/DiagnosticStatus.hpp>

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

#endif  // ROMEA_CORE_LOCALISATION_LOCALISATIONFSMSTATE_HPP_
