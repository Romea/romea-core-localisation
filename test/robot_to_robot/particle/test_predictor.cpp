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
#include "romea_core_localisation/robot_to_robot/particle/predictor.hpp"

namespace
{

using FSMState = romea::core::localisation::FSMState;
using MetaState = romea::core::localisation::R2RPFMetaState;
using Predictor = romea::core::localisation::R2RPFPredictor;

constexpr size_t kNumberOfParticles = 4;

Predictor make_predictor()
{
  return Predictor(romea::core::durationFromSecond(100.0), 100.0, 100.0, kNumberOfParticles);
}

MetaState make_running_state()
{
  MetaState state(kNumberOfParticles);
  state.state.particles.row(MetaState::LEADER_POSITION_X).setConstant(5.0);
  state.state.particles.row(MetaState::LEADER_POSITION_Y).setConstant(1.0);
  state.state.particles.row(MetaState::LEADER_ORIENTATION_Z).setConstant(0.0);
  state.state.weights.setConstant(1.0 / kNumberOfParticles);
  state.input.U().setZero();
  state.input.QU().setZero();
  state.addon.travelled_distance = 0.5;
  state.addon.last_exteroceptive_update.time = romea::core::Duration::zero();
  state.addon.last_exteroceptive_update.travelled_distance = 0.0;
  return state;
}

}  // namespace

TEST(TestR2RPFPredictor, DISABLED_followerForwardMotionPropagatesRelativeLeaderPose)
{
  auto predictor = make_predictor();
  auto previous = make_running_state();
  previous.input.U(MetaState::LINEAR_SPEED_X_BODY) = 1.0;
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
  EXPECT_TRUE((current.state.particles.row(MetaState::LEADER_POSITION_X) == 4.0).all());
  EXPECT_TRUE((current.state.particles.row(MetaState::LEADER_POSITION_Y) == 1.0).all());
  EXPECT_TRUE((current.state.particles.row(MetaState::LEADER_ORIENTATION_Z) == 0.0).all());
  EXPECT_DOUBLE_EQ(current.addon.travelled_distance, 1.5);
}

TEST(TestR2RPFPredictor, DISABLED_leaderBodyMotionPropagatesRelativeLeaderPose)
{
  auto predictor = make_predictor();
  auto previous = make_running_state();
  previous.input.U(MetaState::LEADER_LINEAR_SPEED_X_BODY) = 1.0;
  previous.input.U(MetaState::LEADER_LINEAR_SPEED_Y_BODY) = 2.0;
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
  EXPECT_TRUE((current.state.particles.row(MetaState::LEADER_POSITION_X) == 6.0).all());
  EXPECT_TRUE((current.state.particles.row(MetaState::LEADER_POSITION_Y) == 3.0).all());
  EXPECT_TRUE((current.state.particles.row(MetaState::LEADER_ORIENTATION_Z) == 0.0).all());
}

TEST(TestR2RPFPredictor, deadReckoningLimitsResetState)
{
  Predictor predictor(romea::core::durationFromSecond(0.5), 100.0, 100.0, kNumberOfParticles);
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
  EXPECT_TRUE(std::isnan(current.state.particles(MetaState::LEADER_POSITION_X, 0)));
  EXPECT_TRUE(std::isnan(current.state.particles(MetaState::LEADER_POSITION_Y, 0)));
  EXPECT_TRUE(std::isnan(current.state.particles(MetaState::LEADER_ORIENTATION_Z, 0)));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
