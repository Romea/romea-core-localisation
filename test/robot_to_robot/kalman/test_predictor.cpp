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
#include "romea_core_localisation/robot_to_robot/kalman/predictor.hpp"

namespace
{

using FSMState = romea::core::localisation::FSMState;
using MetaState = romea::core::localisation::R2RKFMetaState;
using Predictor = romea::core::localisation::R2RKFPredictor;

Predictor make_predictor()
{
  return Predictor(romea::core::durationFromSecond(100.0), 100.0, 100.0);
}

MetaState make_state()
{
  MetaState state;
  state.state.X() << 5.0, 1.0, 0.2;
  state.state.P().setIdentity();
  state.input.U().setZero();
  state.input.QU().setIdentity();
  state.input.QU() *= 0.01;
  state.addon.last_exteroceptive_update.time = romea::core::Duration::zero();
  return state;
}

}  // namespace

TEST(TestR2RKFPredictor, zeroMotionKeepsRelativePose)
{
  auto predictor = make_predictor();
  const auto previous = make_state();
  MetaState current;
  FSMState current_fsm_state = FSMState::INIT;

  predictor.predict(
    romea::core::durationFromSecond(1.0),
    FSMState::RUNNING,
    previous,
    romea::core::durationFromSecond(2.0),
    current_fsm_state,
    current);

  EXPECT_EQ(current_fsm_state, FSMState::RUNNING);
  EXPECT_TRUE(current.state.X().isApprox(previous.state.X()));
  EXPECT_TRUE(current.input.U().isApprox(previous.input.U()));
}

TEST(TestR2RKFPredictor, leaderForwardMotionPropagatesRelativePose)
{
  auto predictor = make_predictor();
  auto previous = make_state();
  previous.state.X() << 5.0, 1.0, 0.0;
  previous.input.U(MetaState::LEADER_LINEAR_SPEED_X_BODY) = 1.0;
  previous.input.U(MetaState::LEADER_LINEAR_SPEED_Y_BODY) = 0.0;
  previous.input.U(MetaState::LEADER_ANGULAR_SPEED_Z_BODY) = 0.0;
  previous.input.U(MetaState::LINEAR_SPEED_X_BODY) = 0.0;
  previous.input.U(MetaState::LINEAR_SPEED_Y_BODY) = 0.0;
  previous.input.U(MetaState::ANGULAR_SPEED_Z_BODY) = 0.0;

  MetaState current;
  FSMState current_fsm_state = FSMState::INIT;

  predictor.predict(
    romea::core::durationFromSecond(1.0),
    FSMState::RUNNING,
    previous,
    romea::core::durationFromSecond(2.0),
    current_fsm_state,
    current);

  Eigen::Vector3d expected;
  expected << 6.0, 1.0, 0.0;

  EXPECT_EQ(current_fsm_state, FSMState::RUNNING);
  EXPECT_TRUE(current.state.X().isApprox(expected, 1e-12));
}

TEST(TestR2RKFPredictor, followerForwardMotionPropagatesRelativePose)
{
  auto predictor = make_predictor();
  auto previous = make_state();
  previous.state.X() << 5.0, 1.0, 0.0;
  previous.input.U(MetaState::LEADER_LINEAR_SPEED_X_BODY) = 0.0;
  previous.input.U(MetaState::LEADER_LINEAR_SPEED_Y_BODY) = 0.0;
  previous.input.U(MetaState::LEADER_ANGULAR_SPEED_Z_BODY) = 0.0;
  previous.input.U(MetaState::LINEAR_SPEED_X_BODY) = 1.0;
  previous.input.U(MetaState::LINEAR_SPEED_Y_BODY) = 0.0;
  previous.input.U(MetaState::ANGULAR_SPEED_Z_BODY) = 0.0;

  MetaState current;
  FSMState current_fsm_state = FSMState::INIT;

  predictor.predict(
    romea::core::durationFromSecond(1.0),
    FSMState::RUNNING,
    previous,
    romea::core::durationFromSecond(2.0),
    current_fsm_state,
    current);

  Eigen::Vector3d expected;
  expected << 4.0, 1.0, 0.0;

  EXPECT_EQ(current_fsm_state, FSMState::RUNNING);
  EXPECT_TRUE(current.state.X().isApprox(expected, 1e-12));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
