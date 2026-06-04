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
#include "romea_core_localisation/robot_to_world/kalman/predictor.hpp"

namespace
{

using FSMState = romea::core::localisation::FSMState;
using MetaState = romea::core::localisation::R2WKFMetaState;
using Predictor = romea::core::localisation::R2WKFPredictor;

Predictor make_predictor()
{
  return Predictor(romea::core::durationFromSecond(100.0), 100.0, 100.0);
}

MetaState make_running_state()
{
  MetaState state;
  state.state.X() << 1.0, 2.0, 0.2;
  state.state.P().setIdentity();
  state.input.U() << 1.5, 0.4, 0.1;
  state.input.QU().setIdentity();
  state.input.QU() *= 0.01;
  state.addon.last_exteroceptive_update.time = romea::core::Duration::zero();
  state.addon.last_exteroceptive_update.travelled_distance = 0.0;
  state.addon.travelled_distance = 0.5;
  state.addon.roll = 0.01;
  state.addon.pitch = -0.02;
  state.addon.roll_pitch_variance = 0.03;
  return state;
}

}  // namespace

TEST(TestR2WKFPredictor, nonRunningStateIsCopied)
{
  auto predictor = make_predictor();
  const auto previous = make_running_state();
  MetaState current;
  FSMState current_fsm_state = FSMState::RESET;

  predictor.predict(
    romea::core::durationFromSecond(1.0),
    FSMState::INIT,
    previous,
    romea::core::durationFromSecond(2.0),
    current_fsm_state,
    current);

  EXPECT_EQ(current_fsm_state, FSMState::INIT);
  EXPECT_TRUE(current.state.X().isApprox(previous.state.X()));
  EXPECT_TRUE(current.state.P().isApprox(previous.state.P()));
  EXPECT_TRUE(current.input.U().isApprox(previous.input.U()));
}

TEST(TestR2WKFPredictor, zeroElapsedTimeCopiesRunningState)
{
  auto predictor = make_predictor();
  const auto previous = make_running_state();
  MetaState current;
  FSMState current_fsm_state = FSMState::INIT;
  const auto timestamp = romea::core::durationFromSecond(1.0);

  predictor.predict(timestamp, FSMState::RUNNING, previous, timestamp, current_fsm_state, current);

  EXPECT_EQ(current_fsm_state, FSMState::RUNNING);
  EXPECT_TRUE(current.state.X().isApprox(previous.state.X()));
  EXPECT_TRUE(current.state.P().isApprox(previous.state.P()));
  EXPECT_TRUE(current.input.U().isApprox(previous.input.U()));
}

TEST(TestR2WKFPredictor, runningStateIsPropagated)
{
  auto predictor = make_predictor();
  const auto previous = make_running_state();
  MetaState current;
  FSMState current_fsm_state = FSMState::INIT;

  predictor.predict(
    romea::core::durationFromSecond(1.0),
    FSMState::RUNNING,
    previous,
    romea::core::durationFromSecond(2.0),
    current_fsm_state,
    current);

  const double dt = 1.0;
  const double theta = previous.state.X(MetaState::ORIENTATION_Z);
  const double vx = previous.input.U(MetaState::LINEAR_SPEED_X_BODY);
  const double vy = previous.input.U(MetaState::LINEAR_SPEED_Y_BODY);
  const double w = previous.input.U(MetaState::ANGULAR_SPEED_Z_BODY);
  const double theta_next = theta + w * dt;

  Eigen::Vector3d expected_pose;
  expected_pose << previous.state.X(MetaState::POSITION_X) + vx * dt * std::cos(theta_next) -
                     vy * dt * std::sin(theta_next),
    previous.state.X(MetaState::POSITION_Y) + vx * dt * std::sin(theta_next) +
      vy * dt * std::cos(theta_next),
    theta_next;

  EXPECT_EQ(current_fsm_state, FSMState::RUNNING);
  EXPECT_TRUE(current.state.X().isApprox(expected_pose, 1e-12));
  EXPECT_TRUE(current.input.U().isApprox(previous.input.U()));
  EXPECT_DOUBLE_EQ(
    current.addon.travelled_distance,
    previous.addon.travelled_distance + std::hypot(vx * dt, vy * dt));
  EXPECT_EQ(
    current.addon.last_exteroceptive_update.time, previous.addon.last_exteroceptive_update.time);
}

TEST(TestR2WKFPredictor, deadReckoningLimitsResetState)
{
  Predictor predictor(romea::core::durationFromSecond(0.5), 100.0, 100.0);
  const auto previous = make_running_state();
  MetaState current;
  FSMState current_fsm_state = FSMState::RUNNING;

  predictor.predict(
    romea::core::durationFromSecond(1.0),
    FSMState::RUNNING,
    previous,
    romea::core::durationFromSecond(2.0),
    current_fsm_state,
    current);

  EXPECT_EQ(current_fsm_state, FSMState::INIT);
  EXPECT_TRUE(std::isnan(current.state.X(MetaState::POSITION_X)));
  EXPECT_TRUE(std::isnan(current.state.X(MetaState::POSITION_Y)));
  EXPECT_TRUE(std::isnan(current.state.X(MetaState::ORIENTATION_Z)));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
