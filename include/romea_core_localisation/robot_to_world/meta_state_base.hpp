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

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__META_STATE_BASE_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__META_STATE_BASE_HPP_

// romea
#include <romea_core_common/containers/Eigen/RingOfEigenVector.hpp>
#include <romea_core_filtering/gaussian/input.hpp>

#include "romea_core_localisation/update_monitoring.hpp"

namespace romea {
namespace core {
namespace localisation {

struct R2WMetaStateBase {
  enum StateIndex { POSITION_X = 0, POSITION_Y, ORIENTATION_Z, STATE_SIZE };

  enum InputIndex {
    LINEAR_SPEED_X_BODY = 0,
    LINEAR_SPEED_Y_BODY,
    ANGULAR_SPEED_Z_BODY,
    INPUT_SIZE
  };

  using Input = GaussianInput<double, INPUT_SIZE>;

  struct AddOn {
    AddOn();

    void reset();

    UpdateMonitoring last_exteroceptive_update;

    double roll;
    double pitch;
    double roll_pitch_variance;
    double travelled_distance;
  };

  R2WMetaStateBase();
  virtual ~R2WMetaStateBase() = default;

  Input input;
  AddOn addon;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__META_STATE_BASE_HPP_
