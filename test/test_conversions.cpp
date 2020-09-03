// gtest
#include <gtest/gtest.h>

//romea
#include "test_utils.hpp"
#include "romea_localisation/Pose3D.hpp"

//-----------------------------------------------------------------------------
TEST(TestPose3DConversion, toPose2D)
{
  romea::Pose3D pose3d;
  fillEigenVector(pose3d.position,1);
  fillEigenVector(pose3d.orientation,4);
  fillEigenCovariance(pose3d.covariance);

  romea::Pose2D pose2d = romea::toPose2D(pose3d);

  EXPECT_DOUBLE_EQ(pose2d.yaw,pose3d.orientation.z());
  EXPECT_DOUBLE_EQ(pose2d.position.x(),pose3d.position.x());
  EXPECT_DOUBLE_EQ(pose2d.position.y(),pose3d.position.y());
  EXPECT_DOUBLE_EQ(pose2d.covariance(0,0),pose3d.covariance(0,0));
  EXPECT_DOUBLE_EQ(pose2d.covariance(0,1),pose3d.covariance(0,1));
  EXPECT_DOUBLE_EQ(pose2d.covariance(0,2),pose3d.covariance(0,5));
  EXPECT_DOUBLE_EQ(pose2d.covariance(1,0),pose3d.covariance(1,0));
  EXPECT_DOUBLE_EQ(pose2d.covariance(1,1),pose3d.covariance(1,1));
  EXPECT_DOUBLE_EQ(pose2d.covariance(1,2),pose3d.covariance(1,5));
  EXPECT_DOUBLE_EQ(pose2d.covariance(2,0),pose3d.covariance(5,0));
  EXPECT_DOUBLE_EQ(pose2d.covariance(2,1),pose3d.covariance(5,1));
  EXPECT_DOUBLE_EQ(pose2d.covariance(2,2),pose3d.covariance(5,5));

}

//-----------------------------------------------------------------------------
TEST(TestPose3DConversion, toPosition3D)
{
  romea::Pose3D pose3d;
  fillEigenVector(pose3d.position,1);
  fillEigenVector(pose3d.orientation,4);
  fillEigenCovariance(pose3d.covariance);

  romea::Position3D posisition3d = romea::toPosition3D(pose3d);

  EXPECT_DOUBLE_EQ(posisition3d.position.x(),pose3d.position.x());
  EXPECT_DOUBLE_EQ(posisition3d.position.y(),pose3d.position.y());
  EXPECT_DOUBLE_EQ(posisition3d.position.z(),pose3d.position.z());
  isSame(posisition3d.covariance,Eigen::Matrix3d(pose3d.covariance.block<3,3>(0,0)));
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
