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
#include "romea_core_localisation/robot_to_world/particle/predictor.hpp"

namespace
{

using FSMState = romea::core::localisation::FSMState;
using MetaState = romea::core::localisation::R2WPFMetaState;
using Predictor = romea::core::localisation::R2WPFPredictor;

constexpr size_t kNumberOfParticles = 4;

Predictor make_predictor()
{
  return Predictor(
    kNumberOfParticles,
    romea::core::localisation::DeadReckoningLimits(
      romea::core::durationFromSecond(100.0),
      100.0));
}

MetaState make_running_state()
{
  MetaState state(kNumberOfParticles);
  state.state.particles.row(MetaState::POSITION_X).setConstant(1.0);
  state.state.particles.row(MetaState::POSITION_Y).setConstant(2.0);
  state.state.particles.row(MetaState::ORIENTATION_Z).setConstant(0.0);
  state.state.weights.setConstant(1.0 / kNumberOfParticles);
  state.input.U() << 1.0, 0.0, 0.0;
  state.input.QU().setZero();
  state.addon.roll = 0.1;
  state.addon.pitch = 0.2;
  state.addon.roll_pitch_variance = 0.3;
  state.addon.travelled_distance = 0.5;
  state.addon.dead_reckoning_tracking.start_time = romea::core::Duration::zero();
  state.addon.dead_reckoning_tracking.start_travelled_distance = 0.0;
  return state;
}

}  // namespace

TEST(TestR2WPFPredictor, zeroElapsedTimeCopiesRunningState)
{
  auto predictor = make_predictor();
  const auto previous = make_running_state();
  MetaState current(kNumberOfParticles);
  FSMState current_fsm_state = FSMState::INIT;
  const auto timestamp = romea::core::durationFromSecond(1.0);

  predictor.predict(timestamp, FSMState::RUNNING, previous, timestamp, current_fsm_state, current);

  EXPECT_EQ(current_fsm_state, FSMState::RUNNING);
  EXPECT_TRUE((current.state.particles == previous.state.particles).all());
  EXPECT_TRUE((current.state.weights == previous.state.weights).all());
  EXPECT_TRUE(current.input.U().isApprox(previous.input.U()));
}

TEST(TestR2WPFPredictor, DISABLED_runningStateIsPropagatedWithDeterministicInputs)
{
  auto predictor = make_predictor();
  const auto previous = make_running_state();
  MetaState current(kNumberOfParticles);
  FSMState current_fsm_state = FSMState::INIT;

  predictor.predict(
    romea::core::durationFromSecond(1.0),
    FSMState::RUNNING,
    previous,
    romea::core::durationFromSecond(2.0),
    current_fsm_state,
    current);

  EXPECT_EQ(current_fsm_state, FSMState::RUNNING);
  EXPECT_TRUE((current.state.particles.row(MetaState::POSITION_X) == 2.0).all());
  EXPECT_TRUE((current.state.particles.row(MetaState::POSITION_Y) == 2.0).all());
  EXPECT_TRUE((current.state.particles.row(MetaState::ORIENTATION_Z) == 0.0).all());
  EXPECT_TRUE((current.state.weights == previous.state.weights).all());
  EXPECT_DOUBLE_EQ(current.addon.travelled_distance, 1.5);
  EXPECT_DOUBLE_EQ(current.addon.roll, previous.addon.roll);
  EXPECT_DOUBLE_EQ(current.addon.pitch, previous.addon.pitch);
  EXPECT_DOUBLE_EQ(current.addon.roll_pitch_variance, previous.addon.roll_pitch_variance);
}

TEST(TestR2WPFPredictor, deadReckoningLimitsResetState)
{
  Predictor predictor(
    kNumberOfParticles,
    romea::core::localisation::DeadReckoningLimits(
      romea::core::durationFromSecond(0.5),
      100.0));
  const auto previous = make_running_state();
  MetaState current(kNumberOfParticles);
  FSMState current_fsm_state = FSMState::RUNNING;

  predictor.predict(
    romea::core::durationFromSecond(1.0),
    FSMState::RUNNING,
    previous,
    romea::core::durationFromSecond(2.0),
    current_fsm_state,
    current);

  EXPECT_EQ(current_fsm_state, FSMState::INIT);
  EXPECT_TRUE(std::isnan(current.state.particles(MetaState::POSITION_X, 0)));
  EXPECT_TRUE(std::isnan(current.state.particles(MetaState::POSITION_Y, 0)));
  EXPECT_TRUE(std::isnan(current.state.particles(MetaState::ORIENTATION_Z, 0)));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
