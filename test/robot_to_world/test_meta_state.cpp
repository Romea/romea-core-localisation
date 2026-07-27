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

#include <cmath>

#include "romea_core_common/time/Time.hpp"
#include "romea_core_localisation/robot_to_world/meta_state_base.hpp"

namespace
{

template<typename Input>
void expect_input_is_uninitialized(const Input & input)
{
  for (int i = 0; i < input.U().rows(); ++i) {
    EXPECT_TRUE(std::isnan(input.U(i)));
  }
  EXPECT_TRUE(input.QU().isZero());
}

}  // namespace

TEST(TestR2WMetaStateBase, defaultStateIsConsistent)
{
  romea::core::localisation::R2WMetaStateBase meta_state;

  expect_input_is_uninitialized(meta_state.input);
  EXPECT_EQ(meta_state.addon.dead_reckoning_tracking.start_time, romea::core::Duration::zero());
  EXPECT_TRUE(std::isnan(meta_state.addon.dead_reckoning_tracking.start_travelled_distance));
  for (const auto & time : meta_state.addon.proprioceptive_data_tracking.times) {
    EXPECT_EQ(time, romea::core::Duration::min());
  }
  EXPECT_DOUBLE_EQ(meta_state.addon.roll, 0.0);
  EXPECT_DOUBLE_EQ(meta_state.addon.pitch, 0.0);
  EXPECT_DOUBLE_EQ(meta_state.addon.roll_pitch_variance, 0.0);
  EXPECT_DOUBLE_EQ(meta_state.addon.travelled_distance, 0.0);
}

TEST(TestR2WMetaStateBase, resetClearsRuntimeAddOnData)
{
  romea::core::localisation::R2WMetaStateBase meta_state;
  meta_state.addon.dead_reckoning_tracking.start_time = romea::core::durationFromSecond(42.0);
  meta_state.addon.dead_reckoning_tracking.start_travelled_distance = 12.0;
  meta_state.addon.proprioceptive_data_tracking.times.fill(romea::core::durationFromSecond(42.0));
  meta_state.addon.roll = 0.1;
  meta_state.addon.pitch = 0.2;
  meta_state.addon.roll_pitch_variance = 0.3;
  meta_state.addon.travelled_distance = 4.0;

  meta_state.addon.reset();

  EXPECT_EQ(meta_state.addon.dead_reckoning_tracking.start_time, romea::core::Duration::max());
  EXPECT_DOUBLE_EQ(meta_state.addon.dead_reckoning_tracking.start_travelled_distance, 0.0);
  for (const auto & time : meta_state.addon.proprioceptive_data_tracking.times) {
    EXPECT_EQ(time, romea::core::Duration::min());
  }
  EXPECT_DOUBLE_EQ(meta_state.addon.roll, 0.0);
  EXPECT_DOUBLE_EQ(meta_state.addon.pitch, 0.0);
  EXPECT_DOUBLE_EQ(meta_state.addon.roll_pitch_variance, 0.0);
  EXPECT_DOUBLE_EQ(meta_state.addon.travelled_distance, 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
