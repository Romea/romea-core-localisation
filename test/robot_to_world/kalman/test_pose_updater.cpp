// Copyright 2022 INRAE, French National Research Institute for Agriculture,
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

// gtest
#include <gtest/gtest.h>

#include <memory>

// local
#include "../../test_utils.hpp"
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/robot_to_world/kalman/meta_state.hpp"
#include "romea_core_localisation/robot_to_world/kalman/updater_pose.hpp"

using Updater = romea::core::localisation::R2WKFUpdaterPose;
using FSMState = romea::core::localisation::FSMState;
using MetaState = romea::core::localisation::R2WKFMetaState;
using Observation = romea::core::localisation::ObservationPose;
using trigger_mode = romea::core::localisation::UpdaterTriggerMode;

const Eigen::Vector3d initialPose = (Eigen::Vector3d() << 0.1, 0.2, 0.3).finished();
const Eigen::Matrix3d initialPoseCovariance =
  (Eigen::Matrix3d() << 0.1, 0, 0, 0, 0.2, 0, 0, 0, 0.3).finished();

void set_valid_inputs(MetaState & metastate)
{
  metastate.input.U(MetaState::LINEAR_SPEED_X_BODY) = 1.0;
  metastate.input.U(MetaState::LINEAR_SPEED_Y_BODY) = 0.0;
  metastate.input.U(MetaState::ANGULAR_SPEED_Z_BODY) = 0.1;
}

class TestPoseUpdater : public ::testing::Test
{
public:
  TestPoseUpdater() : metastate(), fsm_state(FSMState::INIT), updater(nullptr) {}

  void init(const FSMState & fsm_state_, const trigger_mode & trigger_mode_)
  {
    updater = std::make_unique<Updater>(
      "course_updater", 100, trigger_mode_, 5);

    metastate.state.X() << initialPose;
    metastate.state.P() << initialPoseCovariance;
    fsm_state = fsm_state_;
  }

  void update(const romea::core::Duration & duration, const Observation & observation)
  {
    updater->update(duration, observation, fsm_state, metastate);
  }

  MetaState metastate;
  FSMState fsm_state;
  std::unique_ptr<Updater> updater;
};

TEST_F(TestPoseUpdater, testSetObservation)
{
  init(FSMState::INIT, trigger_mode::ALWAYS);
  set_valid_inputs(metastate);

  romea::core::Duration duration = romea::core::durationFromSecond(2);

  Observation observation;
  observation.Y() << -0.1, -0.2, 0.4;
  observation.R() << 0.1, 0, 0, 0, 0.2, 0, 0, 0, 0.3;
  updater->update(duration, observation, fsm_state, metastate);

  EXPECT_EQ(fsm_state, FSMState::RUNNING);
  EXPECT_TRUE(metastate.state.X().isApprox(observation.Y()));
  EXPECT_TRUE(metastate.state.P().isApprox(observation.R()));
  EXPECT_EQ(metastate.addon.dead_reckoning_tracking.start_time.count(), duration.count());
  EXPECT_DOUBLE_EQ(metastate.addon.dead_reckoning_tracking.start_travelled_distance, 0);
}

TEST_F(TestPoseUpdater, testUpdate)
{
  init(FSMState::RUNNING, trigger_mode::ALWAYS);

  romea::core::Duration duration = romea::core::durationFromSecond(2);
  Observation observation;
  observation.Y() = initialPose;
  observation.R() = initialPoseCovariance;
  updater->update(duration, observation, fsm_state, metastate);

  EXPECT_EQ(fsm_state, FSMState::RUNNING);
  EXPECT_TRUE(metastate.state.X().isApprox(initialPose));
  EXPECT_TRUE((metastate.state.P().diagonal().isApprox(Eigen::Vector3d(0.05, 0.1, 0.15))));
  EXPECT_EQ(metastate.addon.dead_reckoning_tracking.start_time.count(), duration.count());
  EXPECT_DOUBLE_EQ(metastate.addon.dead_reckoning_tracking.start_travelled_distance, 0);
}

TEST_F(TestPoseUpdater, testMahalanobisRejection)
{
  init(FSMState::RUNNING, trigger_mode::ALWAYS);

  const auto previous_state = metastate.state.X();
  const auto previous_covariance = metastate.state.P();

  romea::core::Duration duration = romea::core::durationFromSecond(2);
  Observation observation;
  observation.Y() << 100, 100, 3;
  observation.R() << 0.1, 0, 0, 0, 0.2, 0, 0, 0, 0.3;
  updater->update(duration, observation, fsm_state, metastate);

  EXPECT_EQ(fsm_state, FSMState::RUNNING);
  EXPECT_TRUE(metastate.state.X().isApprox(previous_state));
  EXPECT_TRUE(metastate.state.P().isApprox(previous_covariance));
  EXPECT_EQ(metastate.addon.dead_reckoning_tracking.start_time, romea::core::Duration::zero());
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
