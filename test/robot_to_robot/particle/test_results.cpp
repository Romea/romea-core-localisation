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
#include "romea_core_localisation/robot_to_robot/particle/results.hpp"

namespace
{

using Results = romea::core::localisation::R2RPFResults;

Results make_results()
{
  Results results(3);
  results.set_duration(romea::core::durationFromSecond(1.0));
  results.state.particles.row(Results::LEADER_POSITION_X) << 1.0, 2.0, 3.0;
  results.state.particles.row(Results::LEADER_POSITION_Y) << 4.0, 5.0, 6.0;
  results.state.particles.row(Results::LEADER_ORIENTATION_Z) << 0.4, 0.4, 0.4;
  results.state.weights.setConstant(1.0 / 3.0);
  results.input.U() << 0.7, -0.2, 0.3, 1.7, -1.2, 1.3;
  results.input.QU().setZero();
  results.input.QU().block<3, 3>(0, 0).setIdentity();
  results.input.QU().block<3, 3>(0, 0) *= 0.01;
  results.input.QU().block<3, 3>(3, 3).setIdentity();
  results.input.QU().block<3, 3>(3, 3) *= 0.02;
  return results;
}

}  // namespace

TEST(TestR2RPFResults, computesWeightedLeaderPoseEstimate)
{
  const auto results = make_results();

  EXPECT_NEAR(results.get_leader_x(), 2.0, 1e-12);
  EXPECT_NEAR(results.get_leader_y(), 5.0, 1e-12);
  EXPECT_NEAR(results.get_leader_orientation(), 0.4, 1e-12);
  EXPECT_TRUE(results.get_leader_pose().isApprox(Eigen::Vector3d(2.0, 5.0, 0.4), 1e-12));
}

TEST(TestR2RPFResults, computesLeaderPoseCovarianceFromParticles)
{
  const auto results = make_results();
  const auto covariance = results.get_leader_pose_covariance();

  EXPECT_NEAR(covariance(Results::LEADER_POSITION_X, Results::LEADER_POSITION_X), 2.0 / 3.0, 1e-12);
  EXPECT_NEAR(covariance(Results::LEADER_POSITION_Y, Results::LEADER_POSITION_Y), 2.0 / 3.0, 1e-12);
  EXPECT_NEAR(covariance(Results::LEADER_ORIENTATION_Z, Results::LEADER_ORIENTATION_Z), 0.0, 1e-12);
}

TEST(TestR2RPFResults, exposesRobotAndLeaderTwists)
{
  const auto results = make_results();
  const auto pose_and_twist = results.to_leader_pose_and_body_twist2d();

  EXPECT_TRUE(results.get_twist().isApprox(results.input.U().segment<3>(0)));
  EXPECT_TRUE(results.get_twist_covariance().isApprox(results.input.QU().block<3, 3>(0, 0)));
  EXPECT_TRUE(results.get_leader_twist().isApprox(results.input.U().segment<3>(3)));
  EXPECT_TRUE(results.get_leader_twist_covariance().isApprox(results.input.QU().block<3, 3>(3, 3)));
  EXPECT_NEAR(pose_and_twist.pose.position.x(), results.get_leader_x(), 1e-12);
  EXPECT_NEAR(pose_and_twist.pose.position.y(), results.get_leader_y(), 1e-12);
  EXPECT_NEAR(pose_and_twist.pose.yaw, results.get_leader_orientation(), 1e-12);
  EXPECT_NEAR(pose_and_twist.twist.linearSpeeds.x(), results.get_leader_linear_speed(), 1e-12);
  EXPECT_NEAR(pose_and_twist.twist.linearSpeeds.y(), results.get_leader_lateral_speed(), 1e-12);
  EXPECT_NEAR(pose_and_twist.twist.angularSpeed, results.get_leader_angular_speed(), 1e-12);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
