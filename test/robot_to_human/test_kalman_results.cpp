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

#include "romea_core_localisation/robot_to_human/kalman/meta_state.hpp"

namespace
{

using Results = romea::core::localisation::R2HResults;
using State = romea::core::localisation::R2HKFMetaState;

Results make_results()
{
  State state;
  state.state.X() << 1.0, 2.0;
  state.state.P() << 0.1, 0.01, 0.01, 0.2;
  state.input.U() << 3.0, 4.0, 0.5;
  state.input.QU() << 0.3, 0.03, 0.04, 0.03, 0.4, 0.05, 0.04, 0.05, 0.5;

  return romea::core::localisation::R2HKFMetaStateToResults().convert(state);
}

}  // namespace

TEST(TestR2HResults, storesLeaderPositionAndRobotTwist)
{
  const auto results = make_results();

  EXPECT_DOUBLE_EQ(results.leader_position.position.x(), 1.0);
  EXPECT_DOUBLE_EQ(results.leader_position.position.y(), 2.0);
  EXPECT_TRUE(results.leader_position.covariance.isApprox(
    (Eigen::Matrix2d() << 0.1, 0.01, 0.01, 0.2).finished()));
  EXPECT_DOUBLE_EQ(results.follower_twist.linearSpeeds.x(), 3.0);
  EXPECT_DOUBLE_EQ(results.follower_twist.linearSpeeds.y(), 4.0);
  EXPECT_DOUBLE_EQ(results.follower_twist.angularSpeed, 0.5);
}

TEST(TestR2HResults, storesFollowerTwistCovariance)
{
  const auto results = make_results();

  EXPECT_TRUE(results.follower_twist.covariance.isApprox(
    (Eigen::Matrix3d() << 0.3, 0.03, 0.04, 0.03, 0.4, 0.05, 0.04, 0.05, 0.5).finished()));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
