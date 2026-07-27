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

#include "romea_core_localisation/robot_to_world/meta_state_base.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
R2WMetaStateBase::AddOn::AddOn()
: dead_reckoning_tracking(),
  proprioceptive_data_tracking(),
  roll(0),
  pitch(0),
  roll_pitch_variance(0),
  travelled_distance(0)
{
  dead_reckoning_tracking.start_time = Duration::zero();
}

//-----------------------------------------------------------------------------
void R2WMetaStateBase::AddOn::reset()
{
  dead_reckoning_tracking.start_time = Duration::max();
  dead_reckoning_tracking.start_travelled_distance = 0;
  proprioceptive_data_tracking = ObservationUpdateTracking<INPUT_SIZE>();
  roll = 0;
  pitch = 0;
  roll_pitch_variance = 0;
  travelled_distance = 0;
}

//-----------------------------------------------------------------------------
R2WMetaStateBase::R2WMetaStateBase() : input(), addon()
{
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
