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
#include "romea_core_localisation/robot_to_robot/kalman/meta_state.hpp"
#include "romea_core_localisation/robot_to_robot/updater_leader_twist.hpp"

namespace
{

using FSMState = romea::core::localisation::FSMState;
using MetaState = romea::core::localisation::R2RKFMetaState;
using Updater = romea::core::localisation::R2RUpdaterLeaderTwist<MetaState>;
using Observation = romea::core::localisation::ObservationTwist;

Observation make_observation()
{
  Observation observation;
  observation.Y() << 1.0, 2.0, 3.0;
  observation.R() << 0.1, 0.01, 0.02, 0.01, 0.2, 0.03, 0.02, 0.03, 0.3;
  return observation;
}

}  // namespace

TEST(TestR2RLeaderTwistUpdater, writesLeaderInputs)
{
  MetaState meta_state;
  FSMState fsm_state = FSMState::INIT;
  Updater updater("leader_twist_updater", 1.0);
  const auto observation = make_observation();

  for (int n = 1; n <= 5; ++n) {
    updater.update(romea::core::durationFromSecond(n), observation, fsm_state, meta_state);
  }

  EXPECT_EQ(fsm_state, FSMState::INIT);
  EXPECT_TRUE(meta_state.input.U()
                .segment<3>(MetaState::LEADER_LINEAR_SPEED_X_BODY)
                .isApprox(observation.Y()));
  EXPECT_TRUE(
    (meta_state.input.QU()
       .block<3, 3>(MetaState::LEADER_LINEAR_SPEED_X_BODY, MetaState::LEADER_LINEAR_SPEED_X_BODY)
       .isApprox(observation.R())));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
