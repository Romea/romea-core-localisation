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

#include "romea_core_localisation/robot_to_human/kalman/results.hpp"

namespace
{

using Results = romea::core::localisation::R2HKFResults;

Results make_results()
{
  Results results;
  results.state.X() << 1.0, 2.0;
  results.state.P() << 0.1, 0.01, 0.01, 0.2;
  results.input.U() << 3.0, 4.0, 0.5;
  results.input.QU() << 0.3, 0.03, 0.04, 0.03, 0.4, 0.05, 0.04, 0.05, 0.5;
  return results;
}

}  // namespace

TEST(TestR2HKFResults, exposesLeaderPositionAndRobotTwistAccessors)
{
  const auto results = make_results();

  EXPECT_DOUBLE_EQ(results.get_leader_x(), 1.0);
  EXPECT_DOUBLE_EQ(results.get_leader_y(), 2.0);
  EXPECT_TRUE(results.get_leader_position().isApprox(results.state.X()));
  EXPECT_TRUE(results.get_leader_position_covariance().isApprox(results.state.P()));
  EXPECT_DOUBLE_EQ(results.get_linear_speed(), 3.0);
  EXPECT_DOUBLE_EQ(results.get_lateral_speed(), 4.0);
  EXPECT_DOUBLE_EQ(results.get_angular_speed(), 0.5);
  EXPECT_TRUE(results.get_twist().isApprox(results.input.U()));
  EXPECT_TRUE(results.get_twist_covariance().isApprox(results.input.QU()));
}

TEST(TestR2HKFResults, convertsToLeaderPosition2D)
{
  const auto results = make_results();
  const auto position = results.to_leader_position2d();

  EXPECT_DOUBLE_EQ(position.position.x(), results.get_leader_x());
  EXPECT_DOUBLE_EQ(position.position.y(), results.get_leader_y());
  EXPECT_TRUE(position.covariance.isApprox(results.get_leader_position_covariance()));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
