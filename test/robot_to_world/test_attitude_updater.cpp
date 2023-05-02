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
#include "romea_core_localisation/LocalisationFSMState.hpp"
#include "romea_core_localisation/robot_to_world/R2WLocalisationUpdaterAttitude.hpp"
#include "romea_core_localisation/robot_to_world/R2WLocalisationMetaState.hpp"

using namespace romea;

TEST(TestAttitudeUpdater, checkUpdate)
{
  ObservationAttitude observation;
  observation.Y(ObservationAttitude::ROLL) = 1;
  observation.Y(ObservationAttitude::PITCH) = 2;
  observation.R(0, 0) = 3;
  observation.R(0, 1) = 0;
  observation.R(1, 0) = 0;
  observation.R(1, 1) = 3;

  Duration t(1000);
  R2WLocalisationMetaState metaState;
  LocalisationFSMState fsmState = LocalisationFSMState::INIT;
  R2WLocalisationUpdaterAttitude<R2WLocalisationMetaState> updater("attitude_updater", 10);

  updater.update(t, observation, fsmState, metaState);

  EXPECT_EQ(fsmState, LocalisationFSMState::INIT);
  EXPECT_EQ(metaState.addon.roll, observation.Y(ObservationAttitude::ROLL));
  EXPECT_EQ(metaState.addon.pitch, observation.Y(ObservationAttitude::PITCH));
  EXPECT_EQ(metaState.addon.rollPitchVariance, observation.R(0, 0));
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
