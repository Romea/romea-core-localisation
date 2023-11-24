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


#ifndef ROMEA_CORE_LOCALISATION__OBSERVATIONLINEARSPEED_HPP_
#define ROMEA_CORE_LOCALISATION__OBSERVATIONLINEARSPEED_HPP_

#include <romea_core_filtering/GaussianObservation.hpp>

namespace romea
{
namespace core
{

using  ObservationLinearSpeed = GaussianObservation<double, 1>;

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__OBSERVATIONLINEARSPEED_HPP_
