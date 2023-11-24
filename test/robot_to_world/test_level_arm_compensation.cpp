// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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


// gtest
#include <gtest/gtest.h>

#include "romea_core_localisation/robot_to_world/R2WLevelArmCompensation.hpp"

class TestLevelArmCompensation : public ::testing::Test
{
public:
  TestLevelArmCompensation()
  : level_arm_compensation(),
    antenna_position(0.5, 1, 2),
    angle_variance(0.1)
  {
  }

  void compute(
    const double & roll,
    const double & pitch,
    const double & yaw)
  {
    level_arm_compensation.compute(
      roll,
      pitch,
      angle_variance,
      yaw,
      angle_variance,
      antenna_position);
  }

  romea::core::LevelArmCompensation level_arm_compensation;
  Eigen::Vector3d antenna_position;
  double angle_variance;
};


TEST_F(TestLevelArmCompensation, roll_compensation)
{
  compute(M_PI_2, 0, 0);
  EXPECT_NEAR(level_arm_compensation.getPosition().x(), antenna_position.x(), 0.0001);
  EXPECT_NEAR(level_arm_compensation.getPosition().y(), -antenna_position.z(), 0.0001);
  EXPECT_NEAR(level_arm_compensation.getPosition().z(), antenna_position.y(), 0.0001);
}

TEST_F(TestLevelArmCompensation, pitch_compensation)
{
  compute(0, M_PI_2, 0);
  EXPECT_NEAR(level_arm_compensation.getPosition().x(), antenna_position.z(), 0.0001);
  EXPECT_NEAR(level_arm_compensation.getPosition().y(), antenna_position.y(), 0.0001);
  EXPECT_NEAR(level_arm_compensation.getPosition().z(), -antenna_position.x(), 0.0001);
}

TEST_F(TestLevelArmCompensation, yaw_compensation)
{
  compute(0, 0, M_PI_2);
  EXPECT_NEAR(level_arm_compensation.getPosition().x(), -antenna_position.y(), 0.0001);
  EXPECT_NEAR(level_arm_compensation.getPosition().y(), antenna_position.x(), 0.0001);
  EXPECT_NEAR(level_arm_compensation.getPosition().z(), antenna_position.z(), 0.0001);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
