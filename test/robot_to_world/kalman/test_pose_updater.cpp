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

// std
#include <memory>

// local
#include "../../test_utils.hpp"
#include "romea_core_localisation/LocalisationFSMState.hpp"
#include "romea_core_localisation/robot_to_world/kalman/R2WLocalisationKFUpdaterPose.hpp"
#include "romea_core_localisation/robot_to_world/R2WLocalisationMetaState.hpp"

using Updater = romea::core::R2WLocalisationKFUpdaterPose;
using FSMState = romea::core::LocalisationFSMState;
using MetaState = romea::core::R2WLocalisationKFMetaState;
using Observation = romea::core::ObservationPose;
using TriggerMode = romea::core::LocalisationUpdaterTriggerMode;

const Eigen::Vector3d initialPose = (Eigen::Vector3d() << 0.1, 0.2, 0.3).finished();
const Eigen::Matrix3d initialPoseCovariance =
  (Eigen::Matrix3d() << 0.1, 0, 0, 0, 0.2, 0, 0, 0, 0.3).finished();

class TestPoseUpdater : public ::testing::Test
{
public:
  TestPoseUpdater()
  : metastate(),
    fsmState(FSMState::INIT),
    updater(nullptr)
  {
  }

  void init(
    const FSMState & fsmState_,
    const TriggerMode & triggerMode_)
  {
    updater = std::make_unique<Updater>(
      "course_updater",
      100,
      triggerMode_,
      5,
      "course_updater.dat");

    metastate.state.X() << initialPose;
    metastate.state.P() << initialPoseCovariance;
    fsmState = fsmState_;
  }

  void update(
    const romea::core::Duration & duration,
    const Observation & observation)
  {
    updater->update(duration, observation, fsmState, metastate);
  }

  MetaState metastate;
  FSMState fsmState;
  std::unique_ptr<Updater> updater;
};

TEST_F(TestPoseUpdater, testSetObservation)
{
  init(FSMState::INIT, TriggerMode::ALWAYS);

  romea::core::Duration duration = romea::core::durationFromSecond(2);

  Observation observation;
  observation.Y() << -0.1, -0.2, 0.4;
  observation.R() << 0.1, 0, 0, 0, 0.2, 0, 0, 0, 0.3;
  updater->update(duration, observation, fsmState, metastate);

  EXPECT_EQ(fsmState, FSMState::INIT);
  isSame(metastate.state.X(), observation.Y());
  isSame(metastate.state.P(), observation.R());
  // EXPECT_EQ(metastate.addon.lastExteroceptiveUpdate.time.count(), duration.count());
  // EXPECT_DOUBLE_EQ(metastate.addon.lastExteroceptiveUpdate.travelledDistance, 0);
}

TEST_F(TestPoseUpdater, testUpdate)
{
  init(FSMState::RUNNING, TriggerMode::ALWAYS);

  romea::core::Duration duration = romea::core::durationFromSecond(2);
  Observation observation;
  observation.Y() = initialPose;
  observation.R() = initialPoseCovariance;
  updater->update(duration, observation, fsmState, metastate);

  std::cout << metastate.state.X() << std::endl;
  std::cout << metastate.state.P() << std::endl;

//  EXPECT_EQ(fsmState,FSMState::RUNNING);
//  EXPECT_EQ(metastate.state.X(MetaState::ORIENTATION_Z),initialCourse);
//  EXPECT_EQ(metastate.state.P(MetaState::ORIENTATION_Z,MetaState::ORIENTATION_Z),0.05);
//  EXPECT_EQ(metastate.addon.lastExteroceptiveUpdate.time.count(),duration.count());
//  EXPECT_DOUBLE_EQ(metastate.addon.lastExteroceptiveUpdate.travelledDistance,0);
}

// TEST_F(TestCourseUpdater, testMahalanobisRejection)
//{
//  init(FSMState::RUNNING,TriggerMode::ALWAYS);

//  romea::Duration duration = romea::durationFromSecond(2);
//  Observation observation;
//  observation.Y()=10;
//  observation.R()=0.1;
//  updater->update(duration,observation,fsmState,metastate);

//  EXPECT_EQ(fsmState,FSMState::RUNNING);
//  EXPECT_EQ(metastate.state.X(MetaState::ORIENTATION_Z),initialCourse);
//  EXPECT_EQ(metastate.state.P(MetaState::ORIENTATION_Z,MetaState::ORIENTATION_Z),initialCourseVariance);
//  EXPECT_EQ(metastate.addon.lastExteroceptiveUpdate.time.count(),duration.count());
//  EXPECT_DOUBLE_EQ(metastate.addon.lastExteroceptiveUpdate.travelledDistance,0);

//}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
