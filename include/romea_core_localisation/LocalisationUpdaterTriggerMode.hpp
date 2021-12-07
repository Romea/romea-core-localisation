#ifndef romea_LocalisationUpdaterTriggerMode_hpp
#define romea_LocalisationUpdaterTriggerMode_hpp

#include <string>

namespace romea {


  enum class LocalisationUpdaterTriggerMode
  {
    ALWAYS,
    ONCE
  };

  std::string toString(const LocalisationUpdaterTriggerMode & triggerMode);

  LocalisationUpdaterTriggerMode toTriggerMode(const std::string & triggerMode);

}//romea

#endif
