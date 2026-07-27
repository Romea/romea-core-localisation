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

#ifndef ROMEA_CORE_LOCALISATION__OBSERVATION_TRACKING_HPP_
#define ROMEA_CORE_LOCALISATION__OBSERVATION_TRACKING_HPP_

// std
#include <algorithm>
#include <array>
#include <cstddef>

// romea
#include <romea_core_common/time/Time.hpp>

namespace romea
{
namespace core
{
namespace localisation
{

template<std::size_t ObservationSize>
struct ObservationUpdateTracking
{
public:
  ObservationUpdateTracking()
  {
    times.fill(Duration::min());
  }

  std::array<Duration, ObservationSize> times;
};

template<std::size_t ObservationSize>
struct ObservationAgeLimits
{
public:
  ObservationAgeLimits()
  {
    maximal_ages.fill(Duration::max());
  }

  template<std::size_t N>
  void update(
    const double & minimal_rate,
    const std::array<std::size_t, N> & input_indexes,
    const double & maximal_age_rate_ratio = 2.0)
  {
    const auto maximal_age =
      minimal_rate > 0.0 ? durationFromSecond(maximal_age_rate_ratio / minimal_rate) :
      Duration::max();

    for (const auto & input_index : input_indexes) {
      maximal_ages[input_index] = std::min(maximal_ages[input_index], maximal_age);
    }
  }

  std::array<Duration, ObservationSize> maximal_ages;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__OBSERVATION_TRACKING_HPP_
