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

#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__PARTICLE__META_STATE_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__PARTICLE__META_STATE_HPP_

// romea
#include <romea_core_filtering/filter/particle/state.hpp>

#include "romea_core_localisation/robot_to_robot/meta_state_base.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

struct R2RPFMetaState : R2RMetaStateBase
{
  explicit R2RPFMetaState(const size_t & number_of_particles);

  virtual ~R2RPFMetaState() = default;

  using State = ParticleFilterState<double, STATE_SIZE>;

  State state;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_ROBOT__PARTICLE__META_STATE_HPP_
