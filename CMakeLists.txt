cmake_minimum_required(VERSION 3.5)

project(
  "romea_localisation"
  VERSION 0.1.0
  #  DESCRIPTION
  #  HOMEPAGE_URL
  LANGUAGES CXX)

find_package(romea_common REQUIRED)
find_package(romea_filtering REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/Pose2D.cpp
  src/Pose3D.cpp
  src/PoseAndTwist2D.cpp
  src/PoseAndTwist3D.cpp
  src/Position2D.cpp
  src/Position3D.cpp
  src/Twist2D.cpp
  src/Twist3D.cpp
  src/LocalisationFSMState.cpp
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

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${romea_common_INCLUDE_DIRS}
  ${romea_filtering_INCLUDE_DIRS}
  )

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall -Wextra -O3 -std=c++17)

target_link_libraries(${PROJECT_NAME} PUBLIC
  romea_common::romea_common)

include(GNUInstallDirs)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(FILES package.xml DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" INSTALL_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")


option(BUILD_TESTS "BUILD WITH TESTS" OFF)
if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif(BUILD_TESTS)
