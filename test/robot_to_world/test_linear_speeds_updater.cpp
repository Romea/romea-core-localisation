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
#include <cmath>

#include "romea_core_localisation/LocalisationFSMState.hpp"
#include "romea_core_localisation/LocalisationUpdaterLinearSpeeds.hpp"
#include "romea_core_localisation/robot_to_world/R2WLocalisationMetaState.hpp"

using namespace romea::core;

TEST(TestAngularSpeedUpdater, checkUpdate)
{
  ObservationLinearSpeeds observation;
  observation.Y(ObservationLinearSpeeds::LINEAR_SPEED_X_BODY) = 1;
  observation.Y(ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY) = 2;
  observation.R(
    ObservationLinearSpeeds::LINEAR_SPEED_X_BODY,
    ObservationLinearSpeeds::LINEAR_SPEED_X_BODY) = 3;
  observation.R(
    ObservationLinearSpeeds::LINEAR_SPEED_X_BODY,
    ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY) = 4;
  observation.R(
    ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY,
    ObservationLinearSpeeds::LINEAR_SPEED_X_BODY) = 5;
  observation.R(
    ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY,
    ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY) = 6;

  Duration t(1000);
  R2WLocalisationMetaState metaState;
  LocalisationFSMState fsmState = LocalisationFSMState::INIT;
  LocalisationUpdaterLinearSpeeds<R2WLocalisationMetaState> updater("twist_updater", 10);

  updater.update(t, observation, fsmState, metaState);

  EXPECT_EQ(fsmState, LocalisationFSMState::INIT);
  EXPECT_EQ(
    metaState.input.U(R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_X_BODY),
    observation.Y(ObservationLinearSpeeds::LINEAR_SPEED_X_BODY));
  EXPECT_EQ(
    metaState.input.U(R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_Y_BODY),
    observation.Y(ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY));
  EXPECT_FALSE(
    std::isfinite(
      metaState.input.U(
        R2WLocalisationMetaState::InputIndex::
        ANGULAR_SPEED_Z_BODY)));

  EXPECT_EQ(
    metaState.input.QU(
      R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_X_BODY,
      R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_X_BODY),
    observation.R(
      ObservationLinearSpeeds::LINEAR_SPEED_X_BODY,
      ObservationLinearSpeeds::LINEAR_SPEED_X_BODY));

  EXPECT_EQ(
    metaState.input.QU(
      R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_X_BODY,
      R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_Y_BODY),
    observation.R(
      ObservationLinearSpeeds::LINEAR_SPEED_X_BODY,
      ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY));


  EXPECT_EQ(
    metaState.input.QU(
      R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_X_BODY,
      R2WLocalisationMetaState::InputIndex::ANGULAR_SPEED_Z_BODY), 0);

  EXPECT_EQ(
    metaState.input.QU(
      R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_Y_BODY,
      R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_X_BODY),
    observation.R(
      ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY,
      ObservationLinearSpeeds::LINEAR_SPEED_X_BODY));

  EXPECT_EQ(
    metaState.input.QU(
      R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_Y_BODY,
      R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_Y_BODY),
    observation.R(
      ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY,
      ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY));

  EXPECT_EQ(
    metaState.input.QU(
      R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_Y_BODY,
      R2WLocalisationMetaState::InputIndex::ANGULAR_SPEED_Z_BODY), 0);

  EXPECT_EQ(
    metaState.input.QU(
      R2WLocalisationMetaState::InputIndex::ANGULAR_SPEED_Z_BODY,
      R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_X_BODY), 0);

  EXPECT_EQ(
    metaState.input.QU(
      R2WLocalisationMetaState::InputIndex::ANGULAR_SPEED_Z_BODY,
      R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_Y_BODY), 0);

  EXPECT_EQ(
    metaState.input.QU(
      R2WLocalisationMetaState::InputIndex::ANGULAR_SPEED_Z_BODY,
      R2WLocalisationMetaState::InputIndex::ANGULAR_SPEED_Z_BODY), 0);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
