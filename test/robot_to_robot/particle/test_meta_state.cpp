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

#include <cmath>

#include "romea_core_localisation/robot_to_robot/particle/meta_state.hpp"

namespace
{

using MetaState = romea::core::localisation::R2RPFMetaState;

void expect_particles_are_reset(const MetaState & meta_state)
{
  for (int row = 0; row < meta_state.state.particles.rows(); ++row) {
    for (int col = 0; col < meta_state.state.particles.cols(); ++col) {
      EXPECT_TRUE(std::isnan(meta_state.state.particles(row, col)));
    }
  }
}

}  // namespace

TEST(TestR2RPFMetaState, constructorInitializesParticleState)
{
  const MetaState meta_state(5);

  EXPECT_EQ(meta_state.state.particles.rows(), MetaState::STATE_SIZE);
  EXPECT_EQ(meta_state.state.particles.cols(), 5);
  EXPECT_EQ(meta_state.state.weights.cols(), 5);
  expect_particles_are_reset(meta_state);
  EXPECT_TRUE((meta_state.state.weights == 0.2).all());
}

TEST(TestR2RPFMetaState, resetRestoresParticlesAndWeights)
{
  MetaState meta_state(5);
  meta_state.state.particles.setConstant(1.0);
  meta_state.state.weights << 0.1, 0.2, 0.3, 0.15, 0.25;

  meta_state.state.reset();

  expect_particles_are_reset(meta_state);
  EXPECT_TRUE((meta_state.state.weights == 0.2).all());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
