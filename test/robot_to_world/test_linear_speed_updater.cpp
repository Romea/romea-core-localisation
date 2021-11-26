﻿// gtest
#include <gtest/gtest.h>
#include <cmath>

#include "romea_localisation/LocalisationFSMState.hpp"
#include "romea_localisation/LocalisationUpdaterLinearSpeed.hpp"
#include "romea_localisation/robot_to_world/R2WLocalisationMetaState.hpp"

using namespace romea;

TEST(TestAngularSpeedUpdater, checkUpdate)
{
  ObservationLinearSpeed observation;
  observation.Y()=1;
  observation.R()=2;

  Duration t(1000);
  R2WLocalisationMetaState metaState;
  LocalisationFSMState fsmState=LocalisationFSMState::INIT;
  LocalisationUpdaterLinearSpeed<R2WLocalisationMetaState> updater("linear_speed_updater",10);

  updater.update(t,observation,fsmState,metaState);

  EXPECT_EQ(fsmState,LocalisationFSMState::INIT);
  EXPECT_EQ(metaState.input.U(R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_X_BODY),observation.Y());
  EXPECT_EQ(metaState.input.U(R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_Y_BODY),0);
  EXPECT_FALSE(std::isfinite(metaState.input.U(R2WLocalisationMetaState::InputIndex::ANGULAR_SPEED_Z_BODY)));

  EXPECT_EQ(metaState.input.QU(R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_X_BODY,
                               R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_X_BODY),observation.R());

  EXPECT_EQ(metaState.input.QU(R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_X_BODY,
                               R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_Y_BODY),0);

  EXPECT_EQ(metaState.input.QU(R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_X_BODY,
                               R2WLocalisationMetaState::InputIndex::ANGULAR_SPEED_Z_BODY),0);

  EXPECT_EQ(metaState.input.QU(R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_Y_BODY,
                               R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_X_BODY),0);

  EXPECT_EQ(metaState.input.QU(R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_Y_BODY,
                               R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_Y_BODY),0);

  EXPECT_EQ(metaState.input.QU(R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_Y_BODY,
                               R2WLocalisationMetaState::InputIndex::ANGULAR_SPEED_Z_BODY),0);

  EXPECT_EQ(metaState.input.QU(R2WLocalisationMetaState::InputIndex::ANGULAR_SPEED_Z_BODY,
                               R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_X_BODY),0);

  EXPECT_EQ(metaState.input.QU(R2WLocalisationMetaState::InputIndex::ANGULAR_SPEED_Z_BODY,
                               R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_Y_BODY),0);

  EXPECT_EQ(metaState.input.QU(R2WLocalisationMetaState::InputIndex::ANGULAR_SPEED_Z_BODY,
                               R2WLocalisationMetaState::InputIndex::ANGULAR_SPEED_Z_BODY),0);


}



//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
