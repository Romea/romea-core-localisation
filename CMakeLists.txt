cmake_minimum_required(VERSION 2.8.3)
project(romea_localisation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Weffc++ -Wall -Wextra -std=c++17 -O3")

#######################
## Find dependencies ##
#######################
set(${PROJECT_NAME}_CATKIN_DEPS romea_common)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
find_package(romea_filtering REQUIRED)

find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})

###########
## Build ##
###########
include_directories(
  include
  SYSTEM ${catkin_INCLUDE_DIRS}
  SYSTEM ${romea_filtering_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  )

add_library(romea_localisation
  src/Pose2D.cpp
  src/Pose3D.cpp
  src/PoseAndTwist2D.cpp
  src/PoseAndTwist3D.cpp
  src/Position2D.cpp
  src/Position3D.cpp
  src/Twist2D.cpp
  src/Twist3D.cpp
  src/LocalisationUpdater.cpp
  src/LocalisationUpdatesMonitoring.cpp
  src/robot_to_human/R2HLocalisationMetaState.cpp
  src/robot_to_human/kalman/R2HLocalisationKFResults.cpp
  src/robot_to_human/kalman/R2HLocalisationKFMetaState.cpp
  src/robot_to_human/kalman/R2HLocalisationKFPredictor.cpp
  src/robot_to_human/kalman/R2HLocalisationKFUpdaterRange.cpp
  src/robot_to_human/kalman/R2HLocalisationKFUpdaterLeaderPosition.cpp
  src/robot_to_robot/R2RLocalisationMetaState.cpp
  src/robot_to_robot/kalman/R2RLocalisationKFResults.cpp
  src/robot_to_robot/kalman/R2RLocalisationKFMetaState.cpp
  src/robot_to_robot/kalman/R2RLocalisationKFPredictor.cpp
  src/robot_to_robot/kalman/R2RLocalisationKFUpdaterRange.cpp
  src/robot_to_robot/kalman/R2RLocalisationKFUpdaterLeaderPose.cpp
  src/robot_to_robot/particle/R2RLocalisationPFResults.cpp
  src/robot_to_robot/particle/R2RLocalisationPFMetaState.cpp
  src/robot_to_robot/particle/R2RLocalisationPFPredictor.cpp
  src/robot_to_robot/particle/R2RLocalisationPFUpdaterRange.cpp
  src/robot_to_robot/particle/R2RLocalisationPFUpdaterLeaderPose.cpp
  src/robot_to_world/R2WLevelArmCompensation.cpp
  src/robot_to_world/R2WLocalisationMetaState.cpp
  src/robot_to_world/kalman/R2WLocalisationKFResults.cpp
  src/robot_to_world/kalman/R2WLocalisationKFMetaState.cpp
  src/robot_to_world/kalman/R2WLocalisationKFPredictor.cpp
  src/robot_to_world/kalman/R2WLocalisationKFUpdaterCourse.cpp
  src/robot_to_world/kalman/R2WLocalisationKFUpdaterRange.cpp
  src/robot_to_world/kalman/R2WLocalisationKFUpdaterPose.cpp
  src/robot_to_world/kalman/R2WLocalisationKFUpdaterPosition.cpp
  src/robot_to_world/particle/R2WLocalisationPFPredictor.cpp
  src/robot_to_world/particle/R2WLocalisationPFMetaState.cpp
  src/robot_to_world/particle/R2WLocalisationPFResults.cpp
  src/robot_to_world/particle/R2WLocalisationPFUpdaterCourse.cpp
  src/robot_to_world/particle/R2WLocalisationPFUpdaterRange.cpp
  src/robot_to_world/particle/R2WLocalisationPFUpdaterPose.cpp
  src/robot_to_world/particle/R2WLocalisationPFUpdaterPosition.cpp
  )

target_link_libraries(romea_localisation ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif(CATKIN_ENABLE_TESTING)
