// romea
#include "romea_core_localisation/LocalisationUpdaterTriggerMode.hpp"

// std
#include <exception>
#include <sstream>

namespace romea {

//-----------------------------------------------------------------------------
std::string toString(const LocalisationUpdaterTriggerMode & triggerMode)
{
  if ( triggerMode==LocalisationUpdaterTriggerMode::ALWAYS)
  {
    return "always";
  } else {
    return "once";
  }
}

//-----------------------------------------------------------------------------
LocalisationUpdaterTriggerMode toTriggerMode(const std::string & triggerMode)
{
  if (triggerMode == "always")
  {
    return LocalisationUpdaterTriggerMode::ALWAYS;
  } else if (triggerMode == "once") {
    return LocalisationUpdaterTriggerMode::ONCE;
  } else {
    std::stringstream msg;
    msg << triggerMode;
    msg << "cannot be converted to LocalisationUpdaterTriggerMode";
    throw(std::runtime_error(msg.str()));
  }
}

}  // namespace romea

