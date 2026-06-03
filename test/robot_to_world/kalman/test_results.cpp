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

#include "romea_core_localisation/robot_to_world/kalman/results.hpp"

namespace
{

using Results = romea::core::localisation::R2WKFResults;

Results make_results()
{
  Results results;
  results.state.X() << 1.0, 2.0, 0.3;
  results.state.P() << 0.1, 0.01, 0.02,
                       0.01, 0.2, 0.03,
                       0.02, 0.03, 0.3;
  results.input.U() << 4.0, 5.0, 0.6;
  results.input.QU() << 0.4, 0.04, 0.05,
                        0.04, 0.5, 0.06,
                        0.05, 0.06, 0.6;
  results.addon.roll = 0.01;
  results.addon.pitch = -0.02;
  results.addon.roll_pitch_variance = 0.03;
  return results;
}

}  // namespace

TEST(TestR2WKFResults, exposesPoseAndTwistAccessors)
{
  const auto results = make_results();

  EXPECT_DOUBLE_EQ(results.get_x(), 1.0);
  EXPECT_DOUBLE_EQ(results.get_y(), 2.0);
  EXPECT_DOUBLE_EQ(results.get_yaw(), 0.3);
  EXPECT_DOUBLE_EQ(results.get_yaw_variance(), 0.3);
  EXPECT_DOUBLE_EQ(results.get_linear_speed(), 4.0);
  EXPECT_DOUBLE_EQ(results.get_lateral_speed(), 5.0);
  EXPECT_DOUBLE_EQ(results.get_angular_speed(), 0.6);
  EXPECT_TRUE(results.get_pose().isApprox(results.state.X()));
  EXPECT_TRUE(results.get_pose_covariance().isApprox(results.state.P()));
  EXPECT_TRUE(results.get_twist().isApprox(results.input.U()));
  EXPECT_TRUE(results.get_twist_covariance().isApprox(results.input.QU()));
}

TEST(TestR2WKFResults, convertsToPose2D)
{
  const auto results = make_results();
  const auto pose = results.to_pose2d();

  EXPECT_DOUBLE_EQ(pose.position.x(), results.get_x());
  EXPECT_DOUBLE_EQ(pose.position.y(), results.get_y());
  EXPECT_DOUBLE_EQ(pose.yaw, results.get_yaw());
  EXPECT_TRUE(pose.covariance.isApprox(results.get_pose_covariance()));
}

TEST(TestR2WKFResults, convertsToPoseAndBodyTwist2D)
{
  const auto results = make_results();
  const auto pose_and_twist = results.to_pose_and_body_twist2d();

  EXPECT_DOUBLE_EQ(pose_and_twist.pose.position.x(), results.get_x());
  EXPECT_DOUBLE_EQ(pose_and_twist.pose.position.y(), results.get_y());
  EXPECT_DOUBLE_EQ(pose_and_twist.pose.yaw, results.get_yaw());
  EXPECT_DOUBLE_EQ(
    pose_and_twist.twist.linearSpeeds.x(),
    results.get_linear_speed());
  EXPECT_DOUBLE_EQ(
    pose_and_twist.twist.linearSpeeds.y(),
    results.get_lateral_speed());
  EXPECT_DOUBLE_EQ(
    pose_and_twist.twist.angularSpeed,
    results.get_angular_speed());
  EXPECT_TRUE(pose_and_twist.pose.covariance.isApprox(results.get_pose_covariance()));
  EXPECT_TRUE(pose_and_twist.twist.covariance.isApprox(results.get_twist_covariance()));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
