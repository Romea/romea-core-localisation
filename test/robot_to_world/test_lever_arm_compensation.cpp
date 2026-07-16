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

// romea
#include "romea_core_common/transform/SmartRotation3D.hpp"
#include "romea_core_localisation/robot_to_world/lever_arm_compensation.hpp"

namespace
{

Eigen::Matrix3d computeNumericalJacobian(
  const Eigen::Vector3d & angles,
  const Eigen::Vector3d & antenna_position)
{
  constexpr double epsilon = 1e-6;

  Eigen::Matrix3d jacobian;
  for (int n = 0; n < 3; ++n) {
    Eigen::Vector3d forward_angles = angles;
    Eigen::Vector3d backward_angles = angles;
    forward_angles(n) += epsilon;
    backward_angles(n) -= epsilon;

    romea::core::SmartRotation3D forward_rotation(forward_angles);
    romea::core::SmartRotation3D backward_rotation(backward_angles);

    jacobian.col(n) =
      (forward_rotation * antenna_position - backward_rotation * antenna_position) / (2 * epsilon);
  }

  return jacobian;
}

}  // namespace

class TestLeverArmCompensation : public ::testing::Test
{
public:
  TestLeverArmCompensation()
  : lever_arm_compensation(), antenna_position(0.5, 1, 2), angle_variance(0.1)
  {
  }

  void compute(const double & roll, const double & pitch, const double & yaw)
  {
    lever_arm_compensation.compute(
      roll, pitch, angle_variance, yaw, angle_variance, antenna_position);
  }

  romea::core::localisation::LeverArmCompensation lever_arm_compensation;
  Eigen::Vector3d antenna_position;
  double angle_variance;
};

TEST_F(TestLeverArmCompensation, roll_compensation)
{
  compute(M_PI_2, 0, 0);
  EXPECT_NEAR(lever_arm_compensation.getPosition().x(), antenna_position.x(), 0.0001);
  EXPECT_NEAR(lever_arm_compensation.getPosition().y(), -antenna_position.z(), 0.0001);
  EXPECT_NEAR(lever_arm_compensation.getPosition().z(), antenna_position.y(), 0.0001);
}

TEST_F(TestLeverArmCompensation, pitch_compensation)
{
  compute(0, M_PI_2, 0);
  EXPECT_NEAR(lever_arm_compensation.getPosition().x(), antenna_position.z(), 0.0001);
  EXPECT_NEAR(lever_arm_compensation.getPosition().y(), antenna_position.y(), 0.0001);
  EXPECT_NEAR(lever_arm_compensation.getPosition().z(), -antenna_position.x(), 0.0001);
}

TEST_F(TestLeverArmCompensation, yaw_compensation)
{
  compute(0, 0, M_PI_2);
  EXPECT_NEAR(lever_arm_compensation.getPosition().x(), -antenna_position.y(), 0.0001);
  EXPECT_NEAR(lever_arm_compensation.getPosition().y(), antenna_position.x(), 0.0001);
  EXPECT_NEAR(lever_arm_compensation.getPosition().z(), antenna_position.z(), 0.0001);
}

TEST_F(TestLeverArmCompensation, jacobian_and_covariance)
{
  const Eigen::Vector3d angles(0.2, -0.3, 0.4);
  const double yaw_variance = 0.2;

  lever_arm_compensation.compute(
    angles.x(), angles.y(), angle_variance, angles.z(), yaw_variance, antenna_position);

  Eigen::Matrix3d expected_jacobian = computeNumericalJacobian(angles, antenna_position);
  Eigen::Matrix3d attitude_covariance = Eigen::Matrix3d::Zero();
  attitude_covariance(0, 0) = angle_variance;
  attitude_covariance(1, 1) = angle_variance;
  attitude_covariance(2, 2) = yaw_variance;

  Eigen::Matrix3d expected_covariance =
    expected_jacobian * attitude_covariance * expected_jacobian.transpose();

  EXPECT_TRUE(lever_arm_compensation.getJacobian().isApprox(expected_jacobian, 1e-5));
  EXPECT_TRUE(lever_arm_compensation.getPositionCovariance().isApprox(expected_covariance, 1e-5));
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
