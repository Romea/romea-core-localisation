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
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/robot_to_world/particle/meta_state.hpp"
#include "romea_core_localisation/robot_to_world/particle/updater_course.hpp"
#include "romea_core_localisation/robot_to_world/particle/updater_pose.hpp"
#include "romea_core_localisation/robot_to_world/particle/updater_position.hpp"

namespace
{

using CourseUpdater = romea::core::localisation::R2WPFUpdaterCourse;
using FSMState = romea::core::localisation::FSMState;
using MetaState = romea::core::localisation::R2WPFMetaState;
using MetaStateToResults = romea::core::localisation::R2WPFMetaStateToResults;
using PoseUpdater = romea::core::localisation::R2WPFUpdaterPose;
using PositionUpdater = romea::core::localisation::R2WPFUpdaterPosition;
using TriggerMode = romea::core::localisation::UpdaterTriggerMode;

constexpr size_t number_of_particles = 20000;
constexpr double x = 12.0;
constexpr double y = -3.0;
constexpr double yaw = 0.25;
constexpr double x_variance = 0.04;
constexpr double y_variance = 0.09;
constexpr double yaw_variance = 0.01;

romea::core::localisation::ObservationCourse make_course_observation()
{
  romea::core::localisation::ObservationCourse observation;
  observation.Y() = yaw;
  observation.R() = yaw_variance;
  return observation;
}

romea::core::localisation::ObservationPosition make_position_observation()
{
  romea::core::localisation::ObservationPosition observation;
  observation.Y() << x, y;
  observation.R() << x_variance, 0.0, 0.0, y_variance;
  observation.lever_arm.setZero();
  return observation;
}

romea::core::localisation::ObservationPose make_pose_observation()
{
  romea::core::localisation::ObservationPose observation;
  observation.Y() << x, y, yaw;
  observation.R() << x_variance, 0.0, 0.0, 0.0, y_variance, 0.0, 0.0, 0.0, yaw_variance;
  observation.lever_arm.setZero();
  return observation;
}

void set_valid_motion_inputs(MetaState & meta_state)
{
  meta_state.input.U(MetaState::LINEAR_SPEED_X_BODY) = 1.0;
  meta_state.input.U(MetaState::LINEAR_SPEED_Y_BODY) = 0.0;
  meta_state.input.U(MetaState::ANGULAR_SPEED_Z_BODY) = 0.1;
  meta_state.input.QU().setIdentity();
}

}  // namespace

TEST(TestR2WPFInitialisationUpdaters, positionAndCourseInitialisationMatchesObservationStatistics)
{
  MetaState meta_state(number_of_particles);
  set_valid_motion_inputs(meta_state);
  FSMState fsm_state = FSMState::INIT;
  CourseUpdater course_updater(
    "course_updater", 100.0, TriggerMode::ALWAYS, number_of_particles, 5.0);
  PositionUpdater position_updater(
    "position_updater", 100.0, TriggerMode::ALWAYS, number_of_particles, 5.0);

  course_updater.update(
    romea::core::durationFromSecond(1.0), make_course_observation(), fsm_state, meta_state);
  ASSERT_EQ(fsm_state, FSMState::INIT);

  position_updater.update(
    romea::core::durationFromSecond(1.01), make_position_observation(), fsm_state, meta_state);
  ASSERT_EQ(fsm_state, FSMState::RUNNING);

  const auto results = MetaStateToResults(number_of_particles).convert(meta_state);

  EXPECT_NEAR(results.robot_pose.position.x(), x, 0.01);
  EXPECT_NEAR(results.robot_pose.position.y(), y, 0.01);
  EXPECT_NEAR(results.robot_pose.orientation.z(), yaw, 0.01);
  EXPECT_NEAR(results.robot_pose.covariance(0, 0), x_variance, 0.005);
  EXPECT_NEAR(results.robot_pose.covariance(1, 1), y_variance, 0.01);
  EXPECT_NEAR(results.robot_pose.covariance(5, 5), yaw_variance, 0.002);
}

TEST(TestR2WPFInitialisationUpdaters, poseInitialisationMatchesObservationStatistics)
{
  MetaState meta_state(number_of_particles);
  set_valid_motion_inputs(meta_state);
  FSMState fsm_state = FSMState::INIT;
  PoseUpdater pose_updater("pose_updater", 100.0, TriggerMode::ALWAYS, number_of_particles, 5.0);

  pose_updater.update(
    romea::core::durationFromSecond(1.0), make_pose_observation(), fsm_state, meta_state);
  ASSERT_EQ(fsm_state, FSMState::RUNNING);

  const auto results = MetaStateToResults(number_of_particles).convert(meta_state);

  EXPECT_NEAR(results.robot_pose.position.x(), x, 0.01);
  EXPECT_NEAR(results.robot_pose.position.y(), y, 0.01);
  EXPECT_NEAR(results.robot_pose.orientation.z(), yaw, 0.01);
  EXPECT_NEAR(results.robot_pose.covariance(0, 0), x_variance, 0.005);
  EXPECT_NEAR(results.robot_pose.covariance(1, 1), y_variance, 0.01);
  EXPECT_NEAR(results.robot_pose.covariance(5, 5), yaw_variance, 0.002);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
