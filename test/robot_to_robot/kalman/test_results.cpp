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

#include "romea_core_localisation/robot_to_robot/kalman/meta_state.hpp"

namespace
{

using Results = romea::core::localisation::R2RResults;
using State = romea::core::localisation::R2RKFMetaState;

Results make_results()
{
  State state;
  state.state.X() << 1.0, 2.0, 0.3;
  state.state.P() << 0.1, 0.01, 0.02, 0.01, 0.2, 0.03, 0.02, 0.03, 0.3;
  state.input.U() << 4.0, 5.0, 0.6, 7.0, 8.0, 0.9;
  state.input.QU().setZero();
  state.input.QU().block<3, 3>(0, 0) << 0.4, 0.04, 0.05, 0.04, 0.5, 0.06, 0.05, 0.06, 0.6;
  state.input.QU().block<3, 3>(3, 3) << 0.7, 0.07, 0.08, 0.07, 0.8, 0.09, 0.08, 0.09, 0.9;

  return romea::core::localisation::R2RKFMetaStateToResults().convert(state);
}

}  // namespace

TEST(TestR2RResults, storesLeaderPoseAndFollowerTwist)
{
  const auto results = make_results();

  EXPECT_DOUBLE_EQ(results.leader_pose_and_twist.pose.position.x(), 1.0);
  EXPECT_DOUBLE_EQ(results.leader_pose_and_twist.pose.position.y(), 2.0);
  EXPECT_DOUBLE_EQ(results.leader_pose_and_twist.pose.yaw, 0.3);
  EXPECT_TRUE(results.leader_pose_and_twist.pose.covariance.isApprox(
    (Eigen::Matrix3d() << 0.1, 0.01, 0.02, 0.01, 0.2, 0.03, 0.02, 0.03, 0.3).finished()));
  EXPECT_DOUBLE_EQ(results.follower_twist.linearSpeeds.x(), 4.0);
  EXPECT_DOUBLE_EQ(results.follower_twist.linearSpeeds.y(), 5.0);
  EXPECT_DOUBLE_EQ(results.follower_twist.angularSpeed, 0.6);
}

TEST(TestR2RResults, storesLeaderTwist)
{
  const auto results = make_results();

  EXPECT_DOUBLE_EQ(results.leader_pose_and_twist.twist.linearSpeeds.x(), 7.0);
  EXPECT_DOUBLE_EQ(results.leader_pose_and_twist.twist.linearSpeeds.y(), 8.0);
  EXPECT_DOUBLE_EQ(results.leader_pose_and_twist.twist.angularSpeed, 0.9);
  EXPECT_TRUE(results.leader_pose_and_twist.twist.covariance.isApprox(
    (Eigen::Matrix3d() << 0.7, 0.07, 0.08, 0.07, 0.8, 0.09, 0.08, 0.09, 0.9).finished()));
}

TEST(TestR2RResults, storesFollowerTwistCovariance)
{
  const auto results = make_results();

  EXPECT_TRUE(results.follower_twist.covariance.isApprox(
    (Eigen::Matrix3d() << 0.4, 0.04, 0.05, 0.04, 0.5, 0.06, 0.05, 0.06, 0.6).finished()));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
