  // gtest
#include <gtest/gtest.h>

#include "romea_core_localisation/robot_to_world/R2WLevelArmCompensation.hpp"

class TestLevelArmCompensation : public ::testing::Test
{
public:
  TestLevelArmCompensation():
    level_arm_compensation(),
    antenna_position(0.5, 1, 2),
    angle_variance(0.1)
  {
  }

  void compute(const double & roll,
               const double & pitch,
               const double & yaw)
  {
    level_arm_compensation.compute(roll,
                                   pitch,
                                   angle_variance,
                                   yaw,
                                   angle_variance,
                                   antenna_position);
  }

  romea::LevelArmCompensation level_arm_compensation;
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
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
