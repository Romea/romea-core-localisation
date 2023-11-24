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


#include "romea_core_localisation/robot_to_world/R2WLocalisationMetaState.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
R2WLocalisationMetaState::AddOn::AddOn()
: lastExteroceptiveUpdate(),
  roll(0),
  pitch(0),
  rollPitchVariance(0),
  travelledDistance(0)
{
}

//-----------------------------------------------------------------------------
void R2WLocalisationMetaState::AddOn::reset()
{
  lastExteroceptiveUpdate.time = Duration::max();
  lastExteroceptiveUpdate.travelledDistance = 0;
  roll = 0;
  pitch = 0;
  rollPitchVariance = 0;
  travelledDistance = 0;
}

//-----------------------------------------------------------------------------
R2WLocalisationMetaState::R2WLocalisationMetaState()
: input(),
  addon()
{
}

}  // namespace core
}  // namespace romea
