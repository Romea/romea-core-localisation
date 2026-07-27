// Copyright 2026 INRAE, French National Research Institute for Agriculture,
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

#include <gtest/gtest.h>

#include "romea_core_common/time/Time.hpp"
#include "romea_core_localisation/robot_to_robot/meta_state_base.hpp"

TEST(TestR2RMetaStateBase, resetClearsRuntimeAddOnData)
{
  romea::core::localisation::R2RMetaStateBase meta_state;
  meta_state.addon.dead_reckoning_tracking.start_time = romea::core::durationFromSecond(42.0);
  meta_state.addon.dead_reckoning_tracking.start_travelled_distance = 12.0;
  meta_state.addon.proprioceptive_data_tracking.times.fill(romea::core::durationFromSecond(42.0));
  meta_state.addon.travelled_distance = 4.0;

  meta_state.addon.reset();

  EXPECT_EQ(meta_state.addon.dead_reckoning_tracking.start_time, romea::core::Duration::max());
  EXPECT_DOUBLE_EQ(meta_state.addon.dead_reckoning_tracking.start_travelled_distance, 0.0);
  for (const auto & time : meta_state.addon.proprioceptive_data_tracking.times) {
    EXPECT_EQ(time, romea::core::Duration::min());
  }
  EXPECT_DOUBLE_EQ(meta_state.addon.travelled_distance, 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
