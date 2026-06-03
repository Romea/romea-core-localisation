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

// std
#include <cmath>
#include <memory>

// romea
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/robot_to_world/kalman/meta_state.hpp"
#include "romea_core_localisation/robot_to_world/kalman/updater_course.hpp"

using Updater = romea::core::localisation::R2WKFUpdaterCourse;
using FSMState = romea::core::localisation::FSMState;
using MetaState = romea::core::localisation::R2WKFMetaState;
using Observation = romea::core::localisation::ObservationCourse;
using trigger_mode = romea::core::localisation::Updatertrigger_mode;

const double initialCourse = 0.1;
const double initialCourseVariance = 0.1;

class TestCourseUpdater : public ::testing::Test {
 public:
  TestCourseUpdater()
      : metastate(), fsm_state(FSMState::INIT), updater(nullptr) {}

  void init(const FSMState& fsm_state_, const trigger_mode& trigger_mode_) {
    updater = std::make_unique<Updater>("course_updater", 100, trigger_mode_, 5,
                                        "course_updater.dat");

    metastate.state.X() << 0, 0, initialCourse;
    metastate.state.P() << 1, 0, 0, 0, 1, 0, 0, 0, initialCourseVariance;
    fsm_state = fsm_state_;
  }

  void update(const romea::core::Duration& duration,
              const Observation& observation) {
    updater->update(duration, observation, fsm_state, metastate);
  }

  MetaState metastate;
  FSMState fsm_state;
  std::unique_ptr<Updater> updater;
};

TEST_F(TestCourseUpdater, testSetObservation) {
  init(FSMState::INIT, trigger_mode::ALWAYS);

  romea::core::Duration duration = romea::core::durationFromSecond(2);

  Observation observation;
  observation.Y() = -0.1;
  observation.R() = 0.1;
  updater->update(duration, observation, fsm_state, metastate);

  EXPECT_EQ(fsm_state, FSMState::INIT);
  EXPECT_EQ(metastate.state.X(MetaState::ORIENTATION_Z), observation.Y());
  EXPECT_EQ(
      metastate.state.P(MetaState::ORIENTATION_Z, MetaState::ORIENTATION_Z),
      observation.R());
  // EXPECT_EQ(metastate.addon.lastExteroceptiveUpdate.time.count(),
  // duration.count());
  // EXPECT_DOUBLE_EQ(metastate.addon.lastExteroceptiveUpdate.travelledDistance,
  // 0);
}

TEST_F(TestCourseUpdater, testUpdate) {
  init(FSMState::RUNNING, trigger_mode::ALWAYS);

  romea::core::Duration duration = romea::core::durationFromSecond(2);
  Observation observation;
  observation.Y() = initialCourse;
  observation.R() = initialCourseVariance;
  updater->update(duration, observation, fsm_state, metastate);

  EXPECT_EQ(fsm_state, FSMState::RUNNING);
  EXPECT_EQ(metastate.state.X(MetaState::ORIENTATION_Z), initialCourse);
  EXPECT_EQ(
      metastate.state.P(MetaState::ORIENTATION_Z, MetaState::ORIENTATION_Z),
      0.05);
  EXPECT_EQ(metastate.addon.last_exteroceptive_update.time.count(),
            duration.count());
  EXPECT_DOUBLE_EQ(metastate.addon.last_exteroceptive_update.travelled_distance,
                   0);
}

TEST_F(TestCourseUpdater, testMahalanobisRejection) {
  init(FSMState::RUNNING, trigger_mode::ALWAYS);

  romea::core::Duration duration = romea::core::durationFromSecond(2);
  Observation observation;
  observation.Y() = 10;
  observation.R() = 0.1;
  updater->update(duration, observation, fsm_state, metastate);

  EXPECT_EQ(fsm_state, FSMState::RUNNING);
  EXPECT_EQ(metastate.state.X(MetaState::ORIENTATION_Z), initialCourse);
  EXPECT_EQ(
      metastate.state.P(MetaState::ORIENTATION_Z, MetaState::ORIENTATION_Z),
      initialCourseVariance);
  EXPECT_EQ(metastate.addon.last_exteroceptive_update.time.count(),
            duration.count());
  EXPECT_DOUBLE_EQ(metastate.addon.last_exteroceptive_update.travelled_distance,
                   0);
}

TEST_F(TestCourseUpdater, testWrappedCourseInnovationIsAccepted) {
  init(FSMState::RUNNING, trigger_mode::ALWAYS);

  const double two_pi = 2.0 * std::acos(-1.0);
  metastate.state.X(MetaState::ORIENTATION_Z) = two_pi - 1e-3;
  metastate.state.P(MetaState::ORIENTATION_Z, MetaState::ORIENTATION_Z) =
      initialCourseVariance;

  romea::core::Duration duration = romea::core::durationFromSecond(2);
  Observation observation;
  observation.Y() = 0.0;
  observation.R() = initialCourseVariance;

  updater->update(duration, observation, fsm_state, metastate);

  EXPECT_EQ(fsm_state, FSMState::RUNNING);
  EXPECT_GT(metastate.state.X(MetaState::ORIENTATION_Z), two_pi - 1e-3);
  EXPECT_LT(metastate.state.X(MetaState::ORIENTATION_Z), two_pi);
  EXPECT_EQ(
      metastate.state.P(MetaState::ORIENTATION_Z, MetaState::ORIENTATION_Z),
      0.05);
  EXPECT_EQ(metastate.addon.last_exteroceptive_update.time.count(),
            duration.count());
}

//-----------------------------------------------------------------------------
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
