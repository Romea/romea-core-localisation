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

#ifndef ROMEA_CORE_LOCALISATION__UPDATER_TRAITS_HPP_
#define ROMEA_CORE_LOCALISATION__UPDATER_TRAITS_HPP_

// std
#include <type_traits>

// romea
#include "romea_core_localisation/updater_exteroceptive.hpp"
#include "romea_core_localisation/updater_proprioceptive.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

template<class Updater>
constexpr bool is_proprioceptive_updater()
{
  return std::is_base_of_v<UpdaterProprioceptive, Updater>;
}

template<class Updater>
constexpr bool is_exteroceptive_updater()
{
  return std::is_base_of_v<UpdaterExteroceptive, Updater>;
}

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__UPDATER_TRAITS_HPP_
