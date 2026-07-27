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

#ifndef ROMEA_CORE_LOCALISATION__DEAD_RECKONING_TRACKING_HPP_
#define ROMEA_CORE_LOCALISATION__DEAD_RECKONING_TRACKING_HPP_

// std
#include <cmath>

// romea
#include <romea_core_common/time/Time.hpp>

namespace romea
{
namespace core
{
namespace localisation
{

struct DeadReckoningTracking
{
public:
  DeadReckoningTracking()
  : start_time(Duration::min()), start_travelled_distance(NAN)
  {
  }

  Duration start_time;
  double start_travelled_distance;
};

struct DeadReckoningLimits
{
public:
  DeadReckoningLimits(
    const Duration & maximal_duration,
    const double & maximal_travelled_distance)
  : maximal_duration(maximal_duration),
    maximal_travelled_distance(maximal_travelled_distance)
  {
  }

  Duration maximal_duration;
  double maximal_travelled_distance;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__DEAD_RECKONING_TRACKING_HPP_
