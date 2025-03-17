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


#ifndef ROMEA_CORE_LOCALISATION__UPDATER_TRIGGER_MODE_HPP_
#define ROMEA_CORE_LOCALISATION__UPDATER_TRIGGER_MODE_HPP_

#include <string>

namespace romea
{
namespace core
{
namespace localisation
{

enum class UpdaterTriggerMode
{
  ALWAYS,
  ONCE
};

std::string to_string(const UpdaterTriggerMode & trigger_mode);

UpdaterTriggerMode to_trigger_mode(const std::string & trigger_mode);

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__UPDATER_TRIGGER_MODE_HPP_
