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


// romea
#include "romea_core_localisation/LocalisationUpdaterTriggerMode.hpp"

// std
#include <exception>
#include <sstream>
#include <string>

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
std::string toString(const LocalisationUpdaterTriggerMode & triggerMode)
{
  if (triggerMode == LocalisationUpdaterTriggerMode::ALWAYS) {
    return "always";
  } else {
    return "once";
  }
}

//-----------------------------------------------------------------------------
LocalisationUpdaterTriggerMode toTriggerMode(const std::string & triggerMode)
{
  if (triggerMode == "always") {
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

}  // namespace core
}  // namespace romea
