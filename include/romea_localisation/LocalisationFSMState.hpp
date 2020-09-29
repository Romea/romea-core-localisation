#ifndef romea_LocalisationFSMState_hpp
#define romea_LocalisationFSMState_hpp

#include <string>
#include <romea_common/diagnostic/DiagnosticStatus.hpp>

namespace romea
{

enum class LocalisationFSMState
{
  INIT=0,
  RUNNING,
  RESET,
  ABORTED
};

std::string toString(const LocalisationFSMState & fmsState);

DiagnosticStatus toDiagnosticStatus(const LocalisationFSMState & fmsState);

}

#endif
