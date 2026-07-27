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

#include <array>
#include <cmath>

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
  return Predictor(
    romea::core::localisation::DeadReckoningLimits(
      romea::core::durationFromSecond(100.0),
      100.0));
}

Predictor make_predictor_with_proprioceptive_age_limits()
{
  Predictor::ObservationAgeLimits observation_age_limits;
  observation_age_limits.update(
    10.0,
    std::array<std::size_t, MetaState::INPUT_SIZE>{
      MetaState::LINEAR_SPEED_X_BODY,
      MetaState::LINEAR_SPEED_Y_BODY,
      MetaState::ANGULAR_SPEED_Z_BODY,
      MetaState::LEADER_LINEAR_SPEED_X_BODY,
      MetaState::LEADER_LINEAR_SPEED_Y_BODY,
      MetaState::LEADER_ANGULAR_SPEED_Z_BODY});

  return Predictor(
    romea::core::localisation::DeadReckoningLimits(
      romea::core::durationFromSecond(100.0),
      100.0),
    observation_age_limits);
}

MetaState make_state()
{
  MetaState state;
  state.state.X() << 5.0, 1.0, 0.2;
  state.state.P().setIdentity();
  state.input.U().setZero();
  state.input.QU().setIdentity();
  state.input.QU() *= 0.01;
  state.addon.dead_reckoning_tracking.start_time = romea::core::Duration::zero();
  state.addon.dead_reckoning_tracking.start_travelled_distance = 0.0;
  return state;
}

MetaState make_static_state_with_proprioceptive_data_at(const romea::core::Duration & duration)
{
  auto state = make_state();
  state.addon.proprioceptive_data_tracking.times.fill(duration);
  return state;
}

void expect_static_state_resets_when_proprioceptive_data_is_lost(
  const std::size_t & lost_input_index)
{
  auto predictor = make_predictor_with_proprioceptive_age_limits();
  const auto initial_time = romea::core::durationFromSecond(1.0);
  const auto fresh_prediction_time = romea::core::durationFromSecond(1.1);
  const auto refreshed_proprioceptive_time = romea::core::durationFromSecond(1.15);
  const auto stale_prediction_time = romea::core::durationFromSecond(1.25);

  const auto previous = make_static_state_with_proprioceptive_data_at(initial_time);
  MetaState current;
  FSMState current_fsm_state = FSMState::INIT;

  predictor.predict(
    initial_time,
    FSMState::RUNNING,
    previous,
    fresh_prediction_time,
    current_fsm_state,
    current);

  EXPECT_EQ(current_fsm_state, FSMState::RUNNING);

  auto previous_after_loss = current;
  previous_after_loss.addon.proprioceptive_data_tracking.times.fill(refreshed_proprioceptive_time);
  previous_after_loss.addon.proprioceptive_data_tracking.times[lost_input_index] = initial_time;

  MetaState current_after_loss;
  predictor.predict(
    refreshed_proprioceptive_time,
    FSMState::RUNNING,
    previous_after_loss,
    stale_prediction_time,
    current_fsm_state,
    current_after_loss);

  EXPECT_EQ(current_fsm_state, FSMState::INIT);
  EXPECT_TRUE(std::isnan(current_after_loss.state.X(MetaState::LEADER_POSITION_X)));
  EXPECT_TRUE(std::isnan(current_after_loss.state.X(MetaState::LEADER_POSITION_Y)));
  EXPECT_TRUE(std::isnan(current_after_loss.state.X(MetaState::LEADER_ORIENTATION_Z)));
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

TEST(TestR2RKFPredictor, staticStateResetsWhenAnyProprioceptiveDataIsLost)
{
  for (const auto & input_index : std::array<std::size_t, MetaState::INPUT_SIZE>{
      MetaState::LINEAR_SPEED_X_BODY,
      MetaState::LINEAR_SPEED_Y_BODY,
      MetaState::ANGULAR_SPEED_Z_BODY,
      MetaState::LEADER_LINEAR_SPEED_X_BODY,
      MetaState::LEADER_LINEAR_SPEED_Y_BODY,
      MetaState::LEADER_ANGULAR_SPEED_Z_BODY}) {
    SCOPED_TRACE(input_index);
    expect_static_state_resets_when_proprioceptive_data_is_lost(input_index);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
