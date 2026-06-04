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

#include "romea_core_common/time/Time.hpp"
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/robot_to_world/kalman/meta_state.hpp"
#include "romea_core_localisation/robot_to_world/kalman/updater_position.hpp"

namespace
{

using FSMState = romea::core::localisation::FSMState;
using MetaState = romea::core::localisation::R2WKFMetaState;
using Observation = romea::core::localisation::ObservationPosition;
using TriggerMode = romea::core::localisation::Updatertrigger_mode;
using Updater = romea::core::localisation::R2WKFUpdaterPosition;

Observation make_position_observation()
{
  Observation observation;
  observation.Y() << 12.0, -3.0;
  observation.R() << 0.4, 0.01, 0.01, 0.5;
  observation.lever_arm.setZero();
  return observation;
}

void set_valid_r2w_initialisation_inputs(MetaState & meta_state)
{
  meta_state.input.U(MetaState::LINEAR_SPEED_X_BODY) = 1.0;
  meta_state.input.U(MetaState::LINEAR_SPEED_Y_BODY) = 0.0;
  meta_state.input.U(MetaState::ANGULAR_SPEED_Z_BODY) = 0.1;
  meta_state.state.X(MetaState::ORIENTATION_Z) = 0.2;
  meta_state.state.P().setIdentity();
}

}  // namespace

TEST(TestR2WKFPositionUpdater, initWaitsForMotionInputsAndCourse)
{
  MetaState meta_state;
  FSMState fsm_state = FSMState::INIT;
  Updater updater("position_updater", 10.0, TriggerMode::ALWAYS, 10.0, "");

  updater.update(
    romea::core::durationFromSecond(1.0), make_position_observation(), fsm_state, meta_state);

  EXPECT_EQ(fsm_state, FSMState::INIT);
  EXPECT_TRUE(std::isnan(meta_state.state.X(MetaState::POSITION_X)));
  EXPECT_TRUE(std::isnan(meta_state.state.X(MetaState::POSITION_Y)));
}

TEST(TestR2WKFPositionUpdater, initSetsPositionAndSwitchesToRunning)
{
  MetaState meta_state;
  set_valid_r2w_initialisation_inputs(meta_state);
  FSMState fsm_state = FSMState::INIT;
  Updater updater("position_updater", 10.0, TriggerMode::ALWAYS, 10.0, "");

  const auto timestamp = romea::core::durationFromSecond(1.5);
  const auto observation = make_position_observation();

  updater.update(timestamp, observation, fsm_state, meta_state);

  EXPECT_EQ(fsm_state, FSMState::RUNNING);
  EXPECT_TRUE(meta_state.state.X().head<2>().isApprox(observation.Y()));
  EXPECT_TRUE((meta_state.state.P().block<2, 2>(0, 0).isApprox(observation.R())));
  EXPECT_EQ(meta_state.addon.last_exteroceptive_update.time, timestamp);
  EXPECT_DOUBLE_EQ(
    meta_state.addon.last_exteroceptive_update.travelled_distance,
    meta_state.addon.travelled_distance);
}

TEST(TestR2WKFPositionUpdater, onceTriggerDoesNotUpdateAfterInitialisation)
{
  MetaState meta_state;
  set_valid_r2w_initialisation_inputs(meta_state);
  FSMState fsm_state = FSMState::INIT;
  Updater updater("position_updater", 10.0, TriggerMode::ONCE, 10.0, "");

  updater.update(
    romea::core::durationFromSecond(1.0), make_position_observation(), fsm_state, meta_state);

  ASSERT_EQ(fsm_state, FSMState::RUNNING);
  const auto state_after_init = meta_state.state.X();

  Observation second_observation = make_position_observation();
  second_observation.Y() << 20.0, 30.0;

  updater.update(romea::core::durationFromSecond(2.0), second_observation, fsm_state, meta_state);

  EXPECT_TRUE(meta_state.state.X().isApprox(state_after_init));
}

TEST(TestR2WKFPositionUpdater, mahalanobisRejectionKeepsPreviousState)
{
  MetaState meta_state;
  meta_state.state.X() << 0.0, 0.0, 0.2;
  meta_state.state.P().setIdentity();
  meta_state.input.U() << 1.0, 0.0, 0.1;
  meta_state.input.QU().setIdentity();
  FSMState fsm_state = FSMState::RUNNING;
  Updater updater("position_updater", 10.0, TriggerMode::ALWAYS, 1.0, "");

  const auto previous_state = meta_state.state.X();
  const auto previous_covariance = meta_state.state.P();

  auto observation = make_position_observation();
  observation.Y() << 100.0, 100.0;
  observation.R().setIdentity();
  observation.R() *= 0.01;

  updater.update(romea::core::durationFromSecond(1.0), observation, fsm_state, meta_state);

  EXPECT_EQ(fsm_state, FSMState::RUNNING);
  EXPECT_TRUE(meta_state.state.X().isApprox(previous_state));
  EXPECT_TRUE(meta_state.state.P().isApprox(previous_covariance));
  EXPECT_EQ(meta_state.addon.last_exteroceptive_update.time, romea::core::Duration::zero());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
