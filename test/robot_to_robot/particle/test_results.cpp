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
#include "romea_core_localisation/robot_to_robot/particle/meta_state.hpp"

namespace
{

using Results = romea::core::localisation::R2RResults;
using State = romea::core::localisation::R2RPFMetaState;

Results make_results()
{
  State state(3);
  state.state.particles.row(State::LEADER_POSITION_X) << 1.0, 2.0, 3.0;
  state.state.particles.row(State::LEADER_POSITION_Y) << 4.0, 5.0, 6.0;
  state.state.particles.row(State::LEADER_ORIENTATION_Z) << 0.4, 0.4, 0.4;
  state.state.weights.setConstant(1.0 / 3.0);
  state.input.U() << 0.7, -0.2, 0.3, 1.7, -1.2, 1.3;
  state.input.QU().setZero();
  state.input.QU().block<3, 3>(0, 0).setIdentity();
  state.input.QU().block<3, 3>(0, 0) *= 0.01;
  state.input.QU().block<3, 3>(3, 3).setIdentity();
  state.input.QU().block<3, 3>(3, 3) *= 0.02;

  return romea::core::localisation::R2RPFMetaStateToResults(3).convert(state);
}

}  // namespace

TEST(TestR2RResults, computesWeightedLeaderPoseEstimate)
{
  const auto results = make_results();

  EXPECT_NEAR(results.leader_pose.position.x(), 2.0, 1e-12);
  EXPECT_NEAR(results.leader_pose.position.y(), 5.0, 1e-12);
  EXPECT_NEAR(results.leader_pose.yaw, 0.4, 1e-12);
}

TEST(TestR2RResults, computesLeaderPoseCovarianceFromParticles)
{
  const auto results = make_results();
  const auto & covariance = results.leader_pose.covariance;

  EXPECT_NEAR(covariance(State::LEADER_POSITION_X, State::LEADER_POSITION_X), 2.0 / 3.0, 1e-12);
  EXPECT_NEAR(covariance(State::LEADER_POSITION_Y, State::LEADER_POSITION_Y), 2.0 / 3.0, 1e-12);
  EXPECT_NEAR(covariance(State::LEADER_ORIENTATION_Z, State::LEADER_ORIENTATION_Z), 0.0, 1e-12);
}

TEST(TestR2RResults, storesFollowerAndLeaderTwists)
{
  const auto results = make_results();

  EXPECT_DOUBLE_EQ(results.follower_twist.linearSpeeds.x(), 0.7);
  EXPECT_DOUBLE_EQ(results.follower_twist.linearSpeeds.y(), -0.2);
  EXPECT_DOUBLE_EQ(results.follower_twist.angularSpeed, 0.3);
  EXPECT_DOUBLE_EQ(results.leader_twist.linearSpeeds.x(), 1.7);
  EXPECT_DOUBLE_EQ(results.leader_twist.linearSpeeds.y(), -1.2);
  EXPECT_DOUBLE_EQ(results.leader_twist.angularSpeed, 1.3);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
