// gtest
#include <gtest/gtest.h>

#include "romea_core_localisation/LocalisationFSMState.hpp"
#include "romea_core_localisation/robot_to_world/kalman/R2WLocalisationKFUpdaterCourse.hpp"
#include "romea_core_localisation/robot_to_world/R2WLocalisationMetaState.hpp"

#include <memory>

using Updater = romea::R2WLocalisationKFUpdaterCourse;
using FSMState = romea::LocalisationFSMState;
using MetaState = romea::R2WLocalisationKFMetaState;
using Observation = romea::ObservationCourse;
using TriggerMode = romea::LocalisationUpdaterTriggerMode ;


const double initialCourse = 0.1;
const double initialCourseVariance = 0.1;

class TestCourseUpdater : public ::testing::Test
{

public:

  TestCourseUpdater():
    metastate(),
    fsmState(FSMState::INIT),
    updater(nullptr)
  {

  }

  void init(const FSMState & fsmState_,
            const TriggerMode & triggerMode_)
  {

    updater = std::make_unique<Updater>("course_updater",
                                        100,
                                        triggerMode_,
                                        5,
                                        "course_updater.dat");

    metastate.state.X() << 0,0,initialCourse;
    metastate.state.P() << 1,0,0,0,1,0,0,0,initialCourseVariance;
    fsmState=fsmState_;


  }

  void update(const romea::Duration & duration,
              const Observation & observation)
  {
    updater->update(duration,observation,fsmState,metastate);
  }

  MetaState metastate;
  FSMState fsmState;
  std::unique_ptr<Updater> updater;
};

TEST_F(TestCourseUpdater, testSetObservation)
{

  init(FSMState::INIT,TriggerMode::ALWAYS);

  romea::Duration duration = romea::durationFromSecond(2);

  Observation observation;
  observation.Y()=-0.1;
  observation.R()=0.1;
  updater->update(duration,observation,fsmState,metastate);

  EXPECT_EQ(fsmState,FSMState::INIT);
  EXPECT_EQ(metastate.state.X(MetaState::ORIENTATION_Z),observation.Y());
  EXPECT_EQ(metastate.state.P(MetaState::ORIENTATION_Z,MetaState::ORIENTATION_Z),observation.R());
  EXPECT_EQ(metastate.addon.lastExteroceptiveUpdate.time.count(),duration.count());
  EXPECT_DOUBLE_EQ(metastate.addon.lastExteroceptiveUpdate.travelledDistance,0);
}

TEST_F(TestCourseUpdater, testUpdate)
{
  init(FSMState::RUNNING,TriggerMode::ALWAYS);

  romea::Duration duration = romea::durationFromSecond(2);
  Observation observation;
  observation.Y()=initialCourse;
  observation.R()=initialCourseVariance;
  updater->update(duration,observation,fsmState,metastate);

  EXPECT_EQ(fsmState,FSMState::RUNNING);
  EXPECT_EQ(metastate.state.X(MetaState::ORIENTATION_Z),initialCourse);
  EXPECT_EQ(metastate.state.P(MetaState::ORIENTATION_Z,MetaState::ORIENTATION_Z),0.05);
  EXPECT_EQ(metastate.addon.lastExteroceptiveUpdate.time.count(),duration.count());
  EXPECT_DOUBLE_EQ(metastate.addon.lastExteroceptiveUpdate.travelledDistance,0);
}

TEST_F(TestCourseUpdater, testMahalanobisRejection)
{
  init(FSMState::RUNNING,TriggerMode::ALWAYS);

  romea::Duration duration = romea::durationFromSecond(2);
  Observation observation;
  observation.Y()=10;
  observation.R()=0.1;
  updater->update(duration,observation,fsmState,metastate);

  EXPECT_EQ(fsmState,FSMState::RUNNING);
  EXPECT_EQ(metastate.state.X(MetaState::ORIENTATION_Z),initialCourse);
  EXPECT_EQ(metastate.state.P(MetaState::ORIENTATION_Z,MetaState::ORIENTATION_Z),initialCourseVariance);
  EXPECT_EQ(metastate.addon.lastExteroceptiveUpdate.time.count(),duration.count());
  EXPECT_DOUBLE_EQ(metastate.addon.lastExteroceptiveUpdate.travelledDistance,0);

}



//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
