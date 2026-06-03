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

#ifndef ROMEA_CORE_LOCALISATION__UPDATER_EXTEROCEPTIVE_HPP_
#define ROMEA_CORE_LOCALISATION__UPDATER_EXTEROCEPTIVE_HPP_

// std
#include <fstream>
#include <string>
#include <vector>

// romea
#include "romea_core_localisation/updater_base.hpp"

namespace romea {
namespace core {
namespace localisation {

class UpdaterExteroceptive : public UpdaterBase {
 public:
  UpdaterExteroceptive(const std::string& updater_name,
                       const double& minimal_rate,
                       const trigger_mode& trigger_mode,
                       const std::string& log_filename);

  virtual ~UpdaterExteroceptive() = default;

  void open_log_file_(const std::string& log_filename);

  void set_log_file_header_(const std::vector<std::string>& log_column_names);

 protected:
  std::ofstream log_file_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__UPDATER_EXTEROCEPTIVE_HPP_
