// gtest
#include <gtest/gtest.h>

#include "romea_localisation/LocalisationFSMState.hpp"
#include "romea_localisation/robot_to_world/R2WLocalisationUpdaterAttitude.hpp"
#include "romea_localisation/robot_to_world/R2WLocalisationMetaState.hpp"

using namespace romea;

TEST(TestAttitudeUpdater, checkUpdate)
{
  ObservationAttitude observation;
  observation.Y(ObservationAttitude::ROLL)=1;
  observation.Y(ObservationAttitude::PITCH)=2;
  observation.R(0,0)=3;
  observation.R(0,1)=0;
  observation.R(1,0)=0;
  observation.R(1,1)=3;

  Duration t(1000);
  R2WLocalisationMetaState metaState;
  LocalisationFSMState fsmState=LocalisationFSMState::INIT;
  R2WLocalisationUpdaterAttitude<R2WLocalisationMetaState> updater;

  updater.update(t,observation,fsmState,metaState);

  EXPECT_EQ(fsmState,LocalisationFSMState::INIT);
  EXPECT_EQ(metaState.addon.roll,observation.Y(ObservationAttitude::ROLL));
  EXPECT_EQ(metaState.addon.pitch,observation.Y(ObservationAttitude::PITCH));
  EXPECT_EQ(metaState.addon.rollPitchVariance,observation.R(0,0));

}



//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
