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

// std
#include <cmath>

// romea
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/robot_to_world/meta_state_base.hpp"
#include "romea_core_localisation/updater_linear_speeds.hpp"

using FSMState = romea::core::localisation::FSMState;
using MetaState = romea::core::localisation::R2WMetaStateBase;
using Observation = romea::core::localisation::ObservationLinearSpeeds;
using Updater = romea::core::localisation::UpdaterLinearSpeeds<MetaState>;

TEST(TestAngularSpeedUpdater, checkUpdate)
{
  Observation observation;
  observation.Y(Observation::LINEAR_SPEED_X_BODY) = 1;
  observation.Y(Observation::LINEAR_SPEED_Y_BODY) = 2;
  observation.R(Observation::LINEAR_SPEED_X_BODY, Observation::LINEAR_SPEED_X_BODY) = 3;
  observation.R(Observation::LINEAR_SPEED_X_BODY, Observation::LINEAR_SPEED_Y_BODY) = 4;
  observation.R(Observation::LINEAR_SPEED_Y_BODY, Observation::LINEAR_SPEED_X_BODY) = 5;
  observation.R(Observation::LINEAR_SPEED_Y_BODY, Observation::LINEAR_SPEED_Y_BODY) = 6;

  MetaState metaState;
  FSMState fsm_state = FSMState::INIT;
  Updater updater("linear_speeds_updater", 1);

  for (int n = 1; n <= 5; ++n) {
    updater.update(romea::core::durationFromSecond(n), observation, fsm_state, metaState);
  }

  EXPECT_EQ(fsm_state, FSMState::INIT);
  EXPECT_EQ(
    metaState.input.U(MetaState::InputIndex::LINEAR_SPEED_X_BODY),
    observation.Y(Observation::LINEAR_SPEED_X_BODY));
  EXPECT_EQ(
    metaState.input.U(MetaState::InputIndex::LINEAR_SPEED_Y_BODY),
    observation.Y(Observation::LINEAR_SPEED_Y_BODY));
  EXPECT_FALSE(std::isfinite(metaState.input.U(MetaState::InputIndex::ANGULAR_SPEED_Z_BODY)));

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::LINEAR_SPEED_X_BODY, MetaState::InputIndex::LINEAR_SPEED_X_BODY),
    observation.R(Observation::LINEAR_SPEED_X_BODY, Observation::LINEAR_SPEED_X_BODY));

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::LINEAR_SPEED_X_BODY, MetaState::InputIndex::LINEAR_SPEED_Y_BODY),
    observation.R(Observation::LINEAR_SPEED_X_BODY, Observation::LINEAR_SPEED_Y_BODY));

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::LINEAR_SPEED_X_BODY, MetaState::InputIndex::ANGULAR_SPEED_Z_BODY),
    0);

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::LINEAR_SPEED_Y_BODY, MetaState::InputIndex::LINEAR_SPEED_X_BODY),
    observation.R(Observation::LINEAR_SPEED_Y_BODY, Observation::LINEAR_SPEED_X_BODY));

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::LINEAR_SPEED_Y_BODY, MetaState::InputIndex::LINEAR_SPEED_Y_BODY),
    observation.R(Observation::LINEAR_SPEED_Y_BODY, Observation::LINEAR_SPEED_Y_BODY));

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::LINEAR_SPEED_Y_BODY, MetaState::InputIndex::ANGULAR_SPEED_Z_BODY),
    0);

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::ANGULAR_SPEED_Z_BODY, MetaState::InputIndex::LINEAR_SPEED_X_BODY),
    0);

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::ANGULAR_SPEED_Z_BODY, MetaState::InputIndex::LINEAR_SPEED_Y_BODY),
    0);

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::ANGULAR_SPEED_Z_BODY, MetaState::InputIndex::ANGULAR_SPEED_Z_BODY),
    0);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
