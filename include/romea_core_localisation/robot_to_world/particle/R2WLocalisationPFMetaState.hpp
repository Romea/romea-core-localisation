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


#ifndef ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__R2WLOCALISATIONPFMETASTATE_HPP_
#define ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__R2WLOCALISATIONPFMETASTATE_HPP_


// romea
#include <romea_core_filtering/particle/ParticleFilterState.hpp>

// std
#include <random>

// local
#include "../R2WLocalisationMetaState.hpp"

namespace romea
{
namespace core
{

struct R2WLocalisationPFMetaState : R2WLocalisationMetaState
{
public:
  using State = ParticleFilterState<double, STATE_SIZE>;

  explicit R2WLocalisationPFMetaState(const size_t & numberOfParticles);

  virtual ~R2WLocalisationPFMetaState() = default;

  State state;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__ROBOT_TO_WORLD__PARTICLE__R2WLOCALISATIONPFMETASTATE_HPP_
