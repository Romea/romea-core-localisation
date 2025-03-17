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

// std
#include <cmath>

// romea
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/updater_angular_speed.hpp"
#include "romea_core_localisation/robot_to_world/meta_state_base.hpp"


using FSMState = romea::core::localisation::FSMState;
using MetaState = romea::core::localisation::R2WMetaStateBase;
using Observation = romea::core::localisation::ObservationAngularSpeed;
using Updater = romea::core::localisation::UpdaterAngularSpeed<MetaState>;

TEST(TestAngularSpeedUpdater, checkUpdate)
{
  Observation observation;
  observation.Y() = 1;
  observation.R() = 2;

  romea::core::Duration t(1000);
  MetaState metaState;
  FSMState fsmState = FSMState::INIT;
  Updater updater("angular_speed_updater", 100);

  updater.update(t, observation, fsmState, metaState);

  EXPECT_EQ(fsmState, FSMState::INIT);
  EXPECT_FALSE(std::isfinite(metaState.input.U(MetaState::InputIndex::LINEAR_SPEED_X_BODY)));
  EXPECT_FALSE(std::isfinite(metaState.input.U(MetaState::InputIndex::LINEAR_SPEED_Y_BODY)));
  EXPECT_EQ(metaState.input.U(MetaState::InputIndex::ANGULAR_SPEED_Z_BODY), observation.Y());

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::LINEAR_SPEED_X_BODY,
      MetaState::InputIndex::LINEAR_SPEED_X_BODY), 0);

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::LINEAR_SPEED_X_BODY,
      MetaState::InputIndex::LINEAR_SPEED_Y_BODY), 0);

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::LINEAR_SPEED_X_BODY,
      MetaState::InputIndex::ANGULAR_SPEED_Z_BODY), 0);

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::LINEAR_SPEED_Y_BODY,
      MetaState::InputIndex::LINEAR_SPEED_X_BODY), 0);

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::LINEAR_SPEED_Y_BODY,
      MetaState::InputIndex::LINEAR_SPEED_Y_BODY), 0);

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::LINEAR_SPEED_Y_BODY,
      MetaState::InputIndex::ANGULAR_SPEED_Z_BODY), 0);

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::ANGULAR_SPEED_Z_BODY,
      MetaState::InputIndex::LINEAR_SPEED_X_BODY), 0);

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::ANGULAR_SPEED_Z_BODY,
      MetaState::InputIndex::LINEAR_SPEED_Y_BODY), 0);

  EXPECT_EQ(
    metaState.input.QU(
      MetaState::InputIndex::ANGULAR_SPEED_Z_BODY,
      MetaState::InputIndex::ANGULAR_SPEED_Z_BODY), observation.R());
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
