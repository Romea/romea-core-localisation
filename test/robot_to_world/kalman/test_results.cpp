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

#include "romea_core_localisation/robot_to_world/kalman/meta_state.hpp"

namespace
{

using Results = romea::core::localisation::R2WResults;
using State = romea::core::localisation::R2WKFMetaState;

Results make_results()
{
  State state;
  state.state.X() << 1.0, 2.0, 0.3;
  state.state.P() << 0.1, 0.01, 0.02, 0.01, 0.2, 0.03, 0.02, 0.03, 0.3;
  state.input.U() << 4.0, 5.0, 0.6;
  state.input.QU() << 0.4, 0.04, 0.05, 0.04, 0.5, 0.06, 0.05, 0.06, 0.6;
  state.addon.roll = 0.01;
  state.addon.pitch = -0.02;
  state.addon.roll_pitch_variance = 0.03;

  return romea::core::localisation::R2WKFMetaStateToResults().convert(state);
}

}  // namespace

TEST(TestR2WResults, storesPoseAndTwist)
{
  const auto results = make_results();

  EXPECT_DOUBLE_EQ(results.robot_pose.position.x(), 1.0);
  EXPECT_DOUBLE_EQ(results.robot_pose.position.y(), 2.0);
  EXPECT_DOUBLE_EQ(results.robot_pose.orientation.x(), 0.01);
  EXPECT_DOUBLE_EQ(results.robot_pose.orientation.y(), -0.02);
  EXPECT_DOUBLE_EQ(results.robot_pose.orientation.z(), 0.3);
  EXPECT_DOUBLE_EQ(results.robot_twist.linearSpeeds.x(), 4.0);
  EXPECT_DOUBLE_EQ(results.robot_twist.linearSpeeds.y(), 5.0);
  EXPECT_DOUBLE_EQ(results.robot_twist.angularSpeeds.z(), 0.6);
}

TEST(TestR2WResults, storesPoseCovariance)
{
  const auto results = make_results();

  EXPECT_DOUBLE_EQ(results.robot_pose.covariance(0, 0), 0.1);
  EXPECT_DOUBLE_EQ(results.robot_pose.covariance(1, 1), 0.2);
  EXPECT_DOUBLE_EQ(results.robot_pose.covariance(3, 3), 0.03);
  EXPECT_DOUBLE_EQ(results.robot_pose.covariance(4, 4), 0.03);
  EXPECT_DOUBLE_EQ(results.robot_pose.covariance(5, 5), 0.3);
}

TEST(TestR2WResults, storesTwistCovariance)
{
  const auto results = make_results();

  EXPECT_DOUBLE_EQ(results.robot_twist.covariance(0, 0), 0.4);
  EXPECT_DOUBLE_EQ(results.robot_twist.covariance(1, 1), 0.5);
  EXPECT_DOUBLE_EQ(results.robot_twist.covariance(5, 5), 0.6);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
