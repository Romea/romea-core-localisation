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
#include "romea_core_localisation/updater_linear_speed.hpp"

using FSMState = romea::core::localisation::FSMState;
using MetaState = romea::core::localisation::R2WMetaStateBase;
using Observation = romea::core::localisation::ObservationLinearSpeed;
using Updater = romea::core::localisation::UpdaterLinearSpeed<MetaState>;

TEST(TestAngularSpeedUpdater, checkUpdate)
{
  Observation observation;
  observation.Y() = 1;
  observation.R() = 2;

  romea::core::Duration t(1000);
  MetaState metaState;
  FSMState fsm_state = FSMState::INIT;
  Updater updater("linear_speed_updater", 10);

  updater.update(t, observation, fsm_state, metaState);

  EXPECT_EQ(fsm_state, FSMState::INIT);
  EXPECT_EQ(metaState.input.U(MetaState::InputIndex::LINEAR_SPEED_X_BODY), observation.Y());
  EXPECT_EQ(metaState.input.U(MetaState::InputIndex::LINEAR_SPEED_Y_BODY), 0);
  EXPECT_FALSE(std::isfinite(metaState.input.U(MetaState::InputIndex::ANGULAR_SPEED_Z_BODY)));

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::LINEAR_SPEED_X_BODY, MetaState::InputIndex::LINEAR_SPEED_X_BODY),
    observation.R());

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::LINEAR_SPEED_X_BODY, MetaState::InputIndex::LINEAR_SPEED_Y_BODY),
    0);

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::LINEAR_SPEED_X_BODY, MetaState::InputIndex::ANGULAR_SPEED_Z_BODY),
    0);

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::LINEAR_SPEED_Y_BODY, MetaState::InputIndex::LINEAR_SPEED_X_BODY),
    0);

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::LINEAR_SPEED_Y_BODY, MetaState::InputIndex::LINEAR_SPEED_Y_BODY),
    0);

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
