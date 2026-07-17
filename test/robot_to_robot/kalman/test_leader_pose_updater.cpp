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
#include "romea_core_localisation/robot_to_robot/kalman/meta_state.hpp"
#include "romea_core_localisation/robot_to_robot/kalman/updater_leader_pose.hpp"

namespace
{

using FSMState = romea::core::localisation::FSMState;
using MetaState = romea::core::localisation::R2RKFMetaState;
using TriggerMode = romea::core::localisation::Updatertrigger_mode;
using Updater = romea::core::localisation::R2RKFUpdaterLeaderPose;
using Observation = romea::core::localisation::ObservationPose;

Observation make_observation()
{
  Observation observation;
  observation.Y() << 4.0, 5.0, 0.6;
  observation.R() << 0.4, 0.01, 0.02, 0.01, 0.5, 0.03, 0.02, 0.03, 0.6;
  return observation;
}

void set_valid_follower_inputs(MetaState & meta_state)
{
  meta_state.input.U(MetaState::LINEAR_SPEED_X_BODY) = 1.0;
  meta_state.input.U(MetaState::LINEAR_SPEED_Y_BODY) = 0.0;
  meta_state.input.U(MetaState::ANGULAR_SPEED_Z_BODY) = 0.1;
}

void set_valid_leader_inputs(MetaState & meta_state)
{
  meta_state.input.U(MetaState::LEADER_LINEAR_SPEED_X_BODY) = 1.5;
  meta_state.input.U(MetaState::LEADER_LINEAR_SPEED_Y_BODY) = 0.2;
  meta_state.input.U(MetaState::LEADER_ANGULAR_SPEED_Z_BODY) = 0.3;
}

}  // namespace

TEST(TestR2RLeaderPoseUpdater, initWaitsForAllFollowerAndLeaderInputs)
{
  MetaState meta_state;
  set_valid_follower_inputs(meta_state);
  meta_state.input.U(MetaState::LEADER_LINEAR_SPEED_X_BODY) = 1.5;

  FSMState fsm_state = FSMState::INIT;
  Updater updater("leader_pose_updater", 10.0, TriggerMode::ALWAYS, 10.0);

  updater.update(romea::core::durationFromSecond(1.0), make_observation(), fsm_state, meta_state);

  EXPECT_EQ(fsm_state, FSMState::INIT);
}

TEST(TestR2RLeaderPoseUpdater, initialisesRelativePose)
{
  MetaState meta_state;
  set_valid_follower_inputs(meta_state);
  set_valid_leader_inputs(meta_state);

  FSMState fsm_state = FSMState::INIT;
  Updater updater("leader_pose_updater", 10.0, TriggerMode::ALWAYS, 10.0);
  const auto timestamp = romea::core::durationFromSecond(1.0);
  const auto observation = make_observation();

  updater.update(timestamp, observation, fsm_state, meta_state);

  EXPECT_EQ(fsm_state, FSMState::RUNNING);
  EXPECT_TRUE(meta_state.state.X().isApprox(observation.Y()));
  EXPECT_TRUE(meta_state.state.P().isApprox(observation.R()));
  EXPECT_EQ(meta_state.addon.last_exteroceptive_update.time, timestamp);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
