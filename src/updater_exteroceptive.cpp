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
#include <string>
#include <vector>

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
  const trigger_mode & trigger_mode,
  const std::string & log_filename)
: UpdaterBase(updater_name, minimal_rate, trigger_mode)
{
  open_log_file_(log_filename);
}

//-----------------------------------------------------------------------------
void UpdaterExteroceptive::open_log_file_(const std::string & log_filename)
{
  if (!log_filename.empty()) {
    log_file_.open(log_filename);

    if (!log_file_.is_open()) {
      throw std::runtime_error("Cannot open debug file : " + log_filename);
    }
  }
}

//-----------------------------------------------------------------------------
void UpdaterExteroceptive::set_log_file_header_(const std::vector<std::string> & log_column_names)
{
  if (log_file_.is_open()) {
    log_file_ << "%";
    for (size_t n = 0; n < log_column_names.size(); ++n) {
      log_file_ << "(" << n + 1 << ")" << log_column_names[n] << ",";
    }
    log_file_ << "\n";
  }
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
