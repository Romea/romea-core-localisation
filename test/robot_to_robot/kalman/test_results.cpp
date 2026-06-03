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

#include "romea_core_localisation/robot_to_robot/kalman/results.hpp"

namespace
{

using Results = romea::core::localisation::R2RKFResults;

Results make_results()
{
  Results results;
  results.state.X() << 1.0, 2.0, 0.3;
  results.state.P() << 0.1, 0.01, 0.02,
                       0.01, 0.2, 0.03,
                       0.02, 0.03, 0.3;
  results.input.U() << 4.0, 5.0, 0.6, 7.0, 8.0, 0.9;
  results.input.QU().setZero();
  results.input.QU().block<3, 3>(0, 0) <<
    0.4, 0.04, 0.05,
    0.04, 0.5, 0.06,
    0.05, 0.06, 0.6;
  results.input.QU().block<3, 3>(3, 3) <<
    0.7, 0.07, 0.08,
    0.07, 0.8, 0.09,
    0.08, 0.09, 0.9;
  return results;
}

}  // namespace

TEST(TestR2RKFResults, exposesLeaderPoseAndRobotTwistAccessors)
{
  const auto results = make_results();

  EXPECT_DOUBLE_EQ(results.get_leader_x(), 1.0);
  EXPECT_DOUBLE_EQ(results.get_leader_y(), 2.0);
  EXPECT_DOUBLE_EQ(results.get_leader_orientation(), 0.3);
  EXPECT_TRUE(results.get_leader_pose().isApprox(results.state.X()));
  EXPECT_TRUE(results.get_leader_pose_covariance().isApprox(results.state.P()));
  EXPECT_DOUBLE_EQ(results.get_linear_speed(), 4.0);
  EXPECT_DOUBLE_EQ(results.get_lateral_speed(), 5.0);
  EXPECT_DOUBLE_EQ(results.get_angular_speed(), 0.6);
  EXPECT_TRUE(results.get_twist().isApprox(results.input.U().segment<3>(0)));
  EXPECT_TRUE(results.get_twist_covariance().isApprox(results.input.QU().block<3, 3>(0, 0)));
}

TEST(TestR2RKFResults, exposesLeaderTwistAccessors)
{
  const auto results = make_results();

  EXPECT_DOUBLE_EQ(results.get_leader_linear_speed(), 7.0);
  EXPECT_DOUBLE_EQ(results.get_leader_lateral_speed(), 8.0);
  EXPECT_DOUBLE_EQ(results.get_leader_angular_speed(), 0.9);
  EXPECT_TRUE(results.get_leader_twist().isApprox(results.input.U().segment<3>(3)));
  EXPECT_TRUE(results.get_leader_twist_covariance().isApprox(results.input.QU().block<3, 3>(3, 3)));
}

TEST(TestR2RKFResults, convertsToLeaderPoseAndBodyTwist2D)
{
  const auto results = make_results();
  const auto pose_and_twist = results.to_leader_pose_and_body_twist2d();

  EXPECT_DOUBLE_EQ(pose_and_twist.pose.position.x(), results.get_leader_x());
  EXPECT_DOUBLE_EQ(pose_and_twist.pose.position.y(), results.get_leader_y());
  EXPECT_DOUBLE_EQ(pose_and_twist.pose.yaw, results.get_leader_orientation());
  EXPECT_TRUE(pose_and_twist.pose.covariance.isApprox(results.get_leader_pose_covariance()));
  EXPECT_DOUBLE_EQ(pose_and_twist.twist.linearSpeeds.x(), results.get_leader_linear_speed());
  EXPECT_DOUBLE_EQ(pose_and_twist.twist.linearSpeeds.y(), results.get_leader_lateral_speed());
  EXPECT_DOUBLE_EQ(pose_and_twist.twist.angularSpeed, results.get_leader_angular_speed());
  EXPECT_TRUE(pose_and_twist.twist.covariance.isApprox(results.get_leader_twist_covariance()));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
