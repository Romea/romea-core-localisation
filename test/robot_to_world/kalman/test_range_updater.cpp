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
#include "romea_core_localisation/robot_to_world/kalman/meta_state.hpp"
#include "romea_core_localisation/robot_to_world/kalman/updater_range.hpp"

namespace
{

using FSMState = romea::core::localisation::FSMState;
using MetaState = romea::core::localisation::R2WKFMetaState;
using TriggerMode = romea::core::localisation::Updatertrigger_mode;
using Updater = romea::core::localisation::R2WKFUpdaterRange;
using Observation = romea::core::localisation::ObservationRange;

MetaState make_running_state()
{
  MetaState meta_state;
  meta_state.state.X() << 0.0, 0.0, 0.0;
  meta_state.state.P().setIdentity();
  meta_state.input.U() << 1.0, 0.0, 0.0;
  meta_state.input.QU().setIdentity();
  return meta_state;
}

Observation make_range_observation(const double & range)
{
  Observation observation;
  observation.Y() = range;
  observation.R() = 0.01;
  observation.initiator_position.setZero();
  observation.responder_position << 1.0, 0.0, 0.0;
  observation.terrain_elevation = 0.0;
  return observation;
}

}  // namespace

TEST(TestR2WKFRangeUpdater, mahalanobisRejectionKeepsPreviousState)
{
  auto meta_state = make_running_state();
  FSMState fsm_state = FSMState::RUNNING;
  Updater updater("range_updater", 10.0, TriggerMode::ALWAYS, 1.0, "");

  const auto previous_state = meta_state.state.X();
  const auto previous_covariance = meta_state.state.P();

  updater.update(
    romea::core::durationFromSecond(1.0), make_range_observation(100.0), fsm_state, meta_state);

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
