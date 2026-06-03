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
#include "romea_core_localisation/robot_to_world/particle/results.hpp"

namespace
{

using Results = romea::core::localisation::R2WPFResults;

Results make_results()
{
  Results results(3);
  results.set_duration(romea::core::durationFromSecond(1.0));
  results.state.particles.row(Results::POSITION_X) << 1.0, 2.0, 3.0;
  results.state.particles.row(Results::POSITION_Y) << 4.0, 5.0, 6.0;
  results.state.particles.row(Results::ORIENTATION_Z) << 0.4, 0.4, 0.4;
  results.state.weights.setConstant(1.0 / 3.0);
  results.input.U() << 0.7, -0.2, 0.3;
  results.input.QU().setIdentity();
  results.input.QU() *= 0.01;
  results.addon.roll = 0.01;
  results.addon.pitch = -0.02;
  results.addon.roll_pitch_variance = 0.03;
  return results;
}

}  // namespace

TEST(TestR2WPFResults, computesWeightedPoseEstimate)
{
  const auto results = make_results();

  EXPECT_NEAR(results.get_x(), 2.0, 1e-12);
  EXPECT_NEAR(results.get_y(), 5.0, 1e-12);
  EXPECT_NEAR(results.get_yaw(), 0.4, 1e-12);
  EXPECT_TRUE(results.get_pose().isApprox(Eigen::Vector3d(2.0, 5.0, 0.4), 1e-12));
}

TEST(TestR2WPFResults, DISABLED_computesPoseCovarianceFromParticles)
{
  const auto results = make_results();
  const auto covariance = results.get_pose_covariance();

  EXPECT_NEAR(covariance(Results::POSITION_X, Results::POSITION_X), 2.0 / 3.0, 1e-12);
  EXPECT_NEAR(covariance(Results::POSITION_Y, Results::POSITION_Y), 2.0 / 3.0, 1e-12);
  EXPECT_NEAR(covariance(Results::ORIENTATION_Z, Results::ORIENTATION_Z), 0.0, 1e-12);
  EXPECT_NEAR(results.get_yaw_variance(), 0.0, 1e-12);
}

TEST(TestR2WPFResults, exposesTwistAndConversions)
{
  const auto results = make_results();
  const auto pose_and_twist = results.to_pose_and_body_twist2d();

  EXPECT_TRUE(results.get_twist().isApprox(results.input.U()));
  EXPECT_TRUE(results.get_twist_covariance().isApprox(results.input.QU()));
  EXPECT_NEAR(pose_and_twist.pose.position.x(), results.get_x(), 1e-12);
  EXPECT_NEAR(pose_and_twist.pose.position.y(), results.get_y(), 1e-12);
  EXPECT_NEAR(pose_and_twist.pose.yaw, results.get_yaw(), 1e-12);
  EXPECT_NEAR(pose_and_twist.twist.linearSpeeds.x(), results.get_linear_speed(), 1e-12);
  EXPECT_NEAR(pose_and_twist.twist.linearSpeeds.y(), results.get_lateral_speed(), 1e-12);
  EXPECT_NEAR(pose_and_twist.twist.angularSpeed, results.get_angular_speed(), 1e-12);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
