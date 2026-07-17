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
#include "romea_core_localisation/robot_to_world/particle/meta_state.hpp"

namespace
{

using Results = romea::core::localisation::R2WResults;
using State = romea::core::localisation::R2WPFMetaState;

Results make_results()
{
  State state(3);
  state.state.particles.row(State::POSITION_X) << 1.0, 2.0, 3.0;
  state.state.particles.row(State::POSITION_Y) << 4.0, 5.0, 6.0;
  state.state.particles.row(State::ORIENTATION_Z) << 0.4, 0.4, 0.4;
  state.state.weights.setConstant(1.0 / 3.0);
  state.input.U() << 0.7, -0.2, 0.3;
  state.input.QU().setIdentity();
  state.input.QU() *= 0.01;
  state.addon.roll = 0.01;
  state.addon.pitch = -0.02;
  state.addon.roll_pitch_variance = 0.03;

  return romea::core::localisation::R2WPFMetaStateToResults(3).convert(state);
}

}  // namespace

TEST(TestR2WResults, computesWeightedPoseEstimate)
{
  const auto results = make_results();

  EXPECT_NEAR(results.robot_pose.position.x(), 2.0, 1e-12);
  EXPECT_NEAR(results.robot_pose.position.y(), 5.0, 1e-12);
  EXPECT_NEAR(results.robot_pose.orientation.z(), 0.4, 1e-12);
}

TEST(TestR2WResults, DISABLED_computesPoseCovarianceFromParticles)
{
  const auto results = make_results();
  const auto & covariance = results.robot_pose.covariance;

  EXPECT_NEAR(covariance(State::POSITION_X, State::POSITION_X), 2.0 / 3.0, 1e-12);
  EXPECT_NEAR(covariance(State::POSITION_Y, State::POSITION_Y), 2.0 / 3.0, 1e-12);
  EXPECT_NEAR(covariance(State::ORIENTATION_Z, State::ORIENTATION_Z), 0.0, 1e-12);
  EXPECT_NEAR(results.robot_pose.covariance(2, 2), 0.0, 1e-12);
}

TEST(TestR2WResults, storesTwist)
{
  const auto results = make_results();

  EXPECT_DOUBLE_EQ(results.robot_twist.linearSpeeds.x(), 0.7);
  EXPECT_DOUBLE_EQ(results.robot_twist.linearSpeeds.y(), -0.2);
  EXPECT_DOUBLE_EQ(results.robot_twist.angularSpeeds.z(), 0.3);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
