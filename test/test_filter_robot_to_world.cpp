// gtest
#include <gtest/gtest.h>

#include <kalman/KalmanFilter.hpp>
#include <filter/robot_to_world/kalman/R2WLocalisationKFResults.hpp>
#include <filter/robot_to_world/kalman/R2WLocalisationKFUpdaterCourse.hpp>
#include <filter/robot_to_world/kalman/R2WLocalisationKFUpdaterRange.hpp>
#include <filter/robot_to_world/kalman/R2WLocalisationKFUpdaterPose.hpp>
#include <filter/robot_to_world/kalman/R2WLocalisationKFUpdaterPosition.hpp>
#include <filter/robot_to_world/kalman/R2WLocalisationKFPredictor.hpp>
#include <filter/robot_to_world/R2WLocalisationUpdaterAttitude.hpp>
#include <filter/LocalisationUpdaterLinearSpeed.hpp>
#include <filter/LocalisationUpdaterLinearSpeeds.hpp>
#include <filter/LocalisationUpdaterAngularSpeed.hpp>

using namespace romea;
using Filter = KalmanFilter<R2WLocalisationKFMetaState,LocalisationFSMState,Duration>;
using UpdaterLinearSpeed = LocalisationUpdaterLinearSpeed<R2WLocalisationKFMetaState>;
using UpdaterLinearSpeeds = LocalisationUpdaterLinearSpeeds<R2WLocalisationKFMetaState>;
using UpdaterAngularSpeed = LocalisationUpdaterAngularSpeed<R2WLocalisationKFMetaState>;
using UpdaterAttitude = R2WLocalisationUpdaterAttitude<R2WLocalisationKFMetaState>;
using UpdaterCourse = R2WLocalisationKFUpdaterCourse;
using UpdaterPose = R2WLocalisationKFUpdaterPose;
using UpdaterPosition = R2WLocalisationKFUpdaterPosition;
using UpdaterRange = R2WLocalisationKFUpdaterRange;
using Predictor = R2WLocalisationKFPredictor;
using Results = R2WLocalisationKFResults;

//-----------------------------------------------------------------------------
TEST(FR2W, toto)
{
  Duration course_stamp = durationFromSecond(31.343000000);
  ObservationCourse course;
  course.firstMoment=0.00174533,0.349066*0.349066);

  Duration position_stamp = durationFromSecond(31.338000000);
  ObservationPosition position()
  Process position
  header:
    seq: 303
    stamp: 31.338000000
    frame_id: map
  position:
    x: 0.388994
    y: 1.21072e-08
    covariance[]
      covariance[0]: 0.0004
      covariance[1]: 0
      covariance[2]: 0
      covariance[3]: 0.0004
    level_arm:
      x: 0
      y: 0
      z: 2

//  romea::R2WLocalisationKFPredictor
//  Process course
//  header:
//    seq: 3
//    stamp: 31.343000000
//    frame_id: robot/gps_link
//  angle: 0.00174533
//  std: 0.349066
}


//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
