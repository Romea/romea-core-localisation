﻿// gtest
#include <gtest/gtest.h>
#include <cmath>

#include "romea_core_localisation/LocalisationFSMState.hpp"
#include "romea_core_localisation/LocalisationUpdaterTwist.hpp"
#include "romea_core_localisation/robot_to_world/R2WLocalisationMetaState.hpp"

using namespace romea;

TEST(TestAngularSpeedUpdater, checkUpdate)
{
  ObservationTwist observation;
  observation.Y(ObservationTwist::LINEAR_SPEED_X_BODY) = 1;
  observation.Y(ObservationTwist::LINEAR_SPEED_Y_BODY) = 2;
  observation.Y(ObservationTwist::ANGULAR_SPEED_Z_BODY) = 3;

  observation.R(ObservationTwist::LINEAR_SPEED_X_BODY,
                ObservationTwist::LINEAR_SPEED_X_BODY) = 4;

  observation.R(ObservationTwist::LINEAR_SPEED_X_BODY,
                ObservationTwist::LINEAR_SPEED_Y_BODY) = 5;

  observation.R(ObservationTwist::LINEAR_SPEED_X_BODY,
                ObservationTwist::ANGULAR_SPEED_Z_BODY) = 6;

  observation.R(ObservationTwist::LINEAR_SPEED_Y_BODY,
                ObservationTwist::LINEAR_SPEED_X_BODY) = 7;

  observation.R(ObservationTwist::LINEAR_SPEED_Y_BODY,
                ObservationTwist::LINEAR_SPEED_Y_BODY) = 8;

  observation.R(ObservationTwist::LINEAR_SPEED_Y_BODY,
                ObservationTwist::ANGULAR_SPEED_Z_BODY) = 9;

  observation.R(ObservationTwist::ANGULAR_SPEED_Z_BODY,
                ObservationTwist::LINEAR_SPEED_X_BODY) = 10;

  observation.R(ObservationTwist::ANGULAR_SPEED_Z_BODY,
                ObservationTwist::LINEAR_SPEED_Y_BODY) = 11;

  observation.R(ObservationTwist::ANGULAR_SPEED_Z_BODY,
                ObservationTwist::ANGULAR_SPEED_Z_BODY) = 12;

  Duration t(1000);
  R2WLocalisationMetaState metaState;
  LocalisationFSMState fsmState = LocalisationFSMState::INIT;
  LocalisationUpdaterTwist<R2WLocalisationMetaState> updater("twist_updater", 10);

  updater.update(t, observation, fsmState, metaState);

  EXPECT_EQ(fsmState, LocalisationFSMState::INIT);

  EXPECT_EQ(metaState.input.U(R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_X_BODY),
             observation.Y(ObservationTwist::LINEAR_SPEED_X_BODY));
  EXPECT_EQ(metaState.input.U(R2WLocalisationMetaState::InputIndex::LINEAR_SPEED_Y_BODY),
             observation.Y(ObservationTwist::LINEAR_SPEED_Y_BODY));
  EXPECT_EQ(metaState.input.U(R2WLocalisationMetaState::InputIndex::ANGULAR_SPEED_Z_BODY),
             observation.Y(ObservationTwist::ANGULAR_SPEED_Z_BODY));
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
