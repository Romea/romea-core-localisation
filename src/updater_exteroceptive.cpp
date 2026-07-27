// Copyright 2022 INRAE, French National Research Institute for Agriculture,
// Food and Environment
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

// std
#include <memory>
#include <utility>

// local
#include "romea_core_localisation/updater_exteroceptive.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
UpdaterExteroceptive::UpdaterExteroceptive(
  const std::string & updater_name,
  const double & minimal_rate,
  const trigger_mode & trigger_mode)
: UpdaterBase(updater_name, minimal_rate, trigger_mode),
  logger_(nullptr)
{
}

//-----------------------------------------------------------------------------
void UpdaterExteroceptive::register_logger(std::shared_ptr<Logger> logger)
{
  logger_ = std::move(logger);
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
