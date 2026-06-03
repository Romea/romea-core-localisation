// Copyright 2022 INRAE, French National Research Institute for Agriculture,
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

// gtest
#include <gtest/gtest.h>

// romea
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/robot_to_world/meta_state_base.hpp"
#include "romea_core_localisation/robot_to_world/updater_attitude.hpp"

using FSMState = romea::core::localisation::FSMState;
using MetaState = romea::core::localisation::R2WMetaStateBase;
using Observation = romea::core::localisation::ObservationAttitude;
using Updater = romea::core::localisation::R2WUpdaterAttitude<MetaState>;

TEST(TestAttitudeUpdater, checkUpdate) {
  Observation observation;
  observation.Y(Observation::ROLL) = 1;
  observation.Y(Observation::PITCH) = 2;
  observation.R(0, 0) = 3;
  observation.R(0, 1) = 0;
  observation.R(1, 0) = 0;
  observation.R(1, 1) = 3;

  romea::core::Duration t(1000);
  MetaState metaState;
  FSMState fsm_state = FSMState::INIT;
  Updater updater("attitude_updater", 10);

  updater.update(t, observation, fsm_state, metaState);

  EXPECT_EQ(fsm_state, FSMState::INIT);
  EXPECT_EQ(metaState.addon.roll, observation.Y(Observation::ROLL));
  EXPECT_EQ(metaState.addon.pitch, observation.Y(Observation::PITCH));
  EXPECT_EQ(metaState.addon.roll_pitch_variance, observation.R(0, 0));
}

//-----------------------------------------------------------------------------
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
