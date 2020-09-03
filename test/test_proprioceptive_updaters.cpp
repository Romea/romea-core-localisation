// gtest
#include <gtest/gtest.h>

#include "test_utils.hpp"
#include "romea_localisation/robot_to_world/R2WLocalisationMetaState.hpp"
#include "romea_localisation/LocalisationUpdaterAngularSpeed.hpp"
#include "romea_localisation/LocalisationUpdaterLinearSpeed.hpp"
#include "romea_localisation/LocalisationUpdaterLinearSpeeds.hpp"
#include "romea_localisation/LocalisationUpdaterTwist.hpp"

using namespace romea;

class TestProprioceptiveUpdater : public ::testing::Test
{

public:

  using MetaState = R2WLocalisationMetaState;

  TestProprioceptiveUpdater():
    fsmState(),
    metaState()
  {
    fillEigenVector(metaState.input.U());
    fillEigenCovariance(metaState.input.QU());
  }

  LocalisationFSMState fsmState;
  R2WLocalisationMetaState metaState;
};

//-----------------------------------------------------------------------------
TEST_F(TestProprioceptiveUpdater,updateLinearSpeed)
{
  ObservationLinearSpeed observation;
  observation.Y()=1;
  observation.R()=2;

  LocalisationUpdaterLinearSpeed<R2WLocalisationMetaState> linear_speed_updater;
  linear_speed_updater.update(Duration::zero(),observation,fsmState,metaState);

  EXPECT_DOUBLE_EQ(metaState.input.U(MetaState::LINEAR_SPEED_X_BODY),observation.Y());
  EXPECT_DOUBLE_EQ(metaState.input.U(MetaState::LINEAR_SPEED_Y_BODY),0);
  EXPECT_DOUBLE_EQ(metaState.input.U(MetaState::ANGULAR_SPEED_Z_BODY),2);

  EXPECT_DOUBLE_EQ(metaState.input.QU(MetaState::LINEAR_SPEED_X_BODY,
                                      MetaState::LINEAR_SPEED_X_BODY),observation.R());

  EXPECT_DOUBLE_EQ(metaState.input.QU(MetaState::LINEAR_SPEED_X_BODY,
                                      MetaState::LINEAR_SPEED_Y_BODY),0);

  EXPECT_DOUBLE_EQ(metaState.input.QU(MetaState::LINEAR_SPEED_X_BODY,
                                      MetaState::ANGULAR_SPEED_Z_BODY),6);

  EXPECT_DOUBLE_EQ(metaState.input.QU(MetaState::LINEAR_SPEED_Y_BODY,
                                      MetaState::LINEAR_SPEED_X_BODY),0);

  EXPECT_DOUBLE_EQ(metaState.input.QU(MetaState::LINEAR_SPEED_Y_BODY,
                                      MetaState::LINEAR_SPEED_Y_BODY),0);

  EXPECT_DOUBLE_EQ(metaState.input.QU(MetaState::LINEAR_SPEED_Y_BODY,
                                      MetaState::ANGULAR_SPEED_Z_BODY),7);

  EXPECT_DOUBLE_EQ(metaState.input.QU(MetaState::ANGULAR_SPEED_Z_BODY,
                                      MetaState::LINEAR_SPEED_X_BODY),2);

  EXPECT_DOUBLE_EQ(metaState.input.QU(MetaState::ANGULAR_SPEED_Z_BODY,
                                      MetaState::LINEAR_SPEED_Y_BODY),5);

  EXPECT_DOUBLE_EQ(metaState.input.QU(MetaState::ANGULAR_SPEED_Z_BODY,
                                      MetaState::ANGULAR_SPEED_Z_BODY),8);


}

//-----------------------------------------------------------------------------
TEST_F(TestProprioceptiveUpdater,updateLinearSpeeds)
{
  ObservationLinearSpeeds observation;
  fillEigenVector(observation.Y(),10);
  fillEigenCovariance(observation.R(),10);

  LocalisationUpdaterLinearSpeeds<R2WLocalisationMetaState> linear_speeds_updater;
  linear_speeds_updater.update(Duration::zero(),observation,fsmState,metaState);

  EXPECT_DOUBLE_EQ(metaState.input.U(MetaState::LINEAR_SPEED_X_BODY),
                   observation.Y(ObservationLinearSpeeds::LINEAR_SPEED_X_BODY));
  EXPECT_DOUBLE_EQ(metaState.input.U(MetaState::LINEAR_SPEED_Y_BODY),
                   observation.Y(ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY));
  EXPECT_DOUBLE_EQ(metaState.input.U(MetaState::ANGULAR_SPEED_Z_BODY),2);

  EXPECT_DOUBLE_EQ(metaState.input.QU(MetaState::LINEAR_SPEED_X_BODY,
                                      MetaState::LINEAR_SPEED_X_BODY),
                   observation.R(ObservationLinearSpeeds::LINEAR_SPEED_X_BODY,
                                 ObservationLinearSpeeds::LINEAR_SPEED_X_BODY));

  EXPECT_DOUBLE_EQ(metaState.input.QU(MetaState::LINEAR_SPEED_X_BODY,
                                      MetaState::LINEAR_SPEED_Y_BODY),
                   observation.R(ObservationLinearSpeeds::LINEAR_SPEED_X_BODY,
                                 ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY));

  EXPECT_DOUBLE_EQ(metaState.input.QU(MetaState::LINEAR_SPEED_X_BODY,
                                      MetaState::ANGULAR_SPEED_Z_BODY),6);

  EXPECT_DOUBLE_EQ(metaState.input.QU(MetaState::LINEAR_SPEED_Y_BODY,
                                      MetaState::LINEAR_SPEED_X_BODY),
                   observation.R(ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY,
                                 ObservationLinearSpeeds::LINEAR_SPEED_X_BODY));

  EXPECT_DOUBLE_EQ(metaState.input.QU(MetaState::LINEAR_SPEED_Y_BODY,
                                      MetaState::LINEAR_SPEED_Y_BODY),
                   observation.R(ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY,
                                 ObservationLinearSpeeds::LINEAR_SPEED_Y_BODY));

  EXPECT_DOUBLE_EQ(metaState.input.QU(MetaState::LINEAR_SPEED_Y_BODY,
                                      MetaState::ANGULAR_SPEED_Z_BODY),7);

  EXPECT_DOUBLE_EQ(metaState.input.QU(MetaState::ANGULAR_SPEED_Z_BODY,
                                      MetaState::LINEAR_SPEED_X_BODY),2);

  EXPECT_DOUBLE_EQ(metaState.input.QU(MetaState::ANGULAR_SPEED_Z_BODY,
                                      MetaState::LINEAR_SPEED_Y_BODY),5);

  EXPECT_DOUBLE_EQ(metaState.input.QU(MetaState::ANGULAR_SPEED_Z_BODY,
                                      MetaState::ANGULAR_SPEED_Z_BODY),8);
}

//-----------------------------------------------------------------------------
TEST_F(TestProprioceptiveUpdater,angularSpeed)
{
  ObservationAngularSpeed observation;
  observation.Y()=1;
  observation.R()=2;

  LocalisationUpdaterAngularSpeed<R2WLocalisationMetaState> angular_speed_updater;
  angular_speed_updater.update(Duration::zero(),observation,fsmState,metaState);

  EXPECT_DOUBLE_EQ(metaState.input.U(MetaState::LINEAR_SPEED_X_BODY),0);
  EXPECT_DOUBLE_EQ(metaState.input.U(MetaState::LINEAR_SPEED_Y_BODY),1);
  EXPECT_DOUBLE_EQ(metaState.input.U(MetaState::ANGULAR_SPEED_Z_BODY),observation.Y());
  EXPECT_DOUBLE_EQ(metaState.input.QU(MetaState::ANGULAR_SPEED_Z_BODY,
                                      MetaState::ANGULAR_SPEED_Z_BODY),observation.R());

  for(size_t n=0;n<8;++n)
  {
    EXPECT_DOUBLE_EQ(metaState.input.QU()(n),n);
  }
}

//-----------------------------------------------------------------------------
TEST_F(TestProprioceptiveUpdater,updateTwist)
{
  ObservationTwist observation;
  fillEigenVector(observation.Y());
  fillEigenCovariance(observation.R());

  LocalisationUpdaterTwist<R2WLocalisationMetaState> twist_updater;
  twist_updater.update(Duration::zero(),observation,fsmState,metaState);
  isSame(metaState.input.U(),observation.Y());
  isSame(metaState.input.QU(),observation.R());
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
