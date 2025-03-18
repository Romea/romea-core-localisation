cmake_minimum_required(VERSION 3.5)

project(
  "romea_core_localisation"
  VERSION 0.1.0
  #  DESCRIPTION
  #  HOMEPAGE_URL
  LANGUAGES CXX)

find_package(romea_core_common REQUIRED)
find_package(romea_core_filtering REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/fsm_state.cpp
  src/updater_base.cpp
  src/updater_exteroceptive.cpp
  src/updater_proprioceptive.cpp
  src/updater_trigger_mode.cpp
  src/update_monitoring.cpp
  src/robot_to_human/meta_state_base.cpp
  src/robot_to_human/kalman/results.cpp
  src/robot_to_human/kalman/meta_state.cpp
  src/robot_to_human/kalman/predictor.cpp
  src/robot_to_human/kalman/updater_range.cpp
  src/robot_to_human/kalman/updater_leader_position.cpp
  src/robot_to_robot/meta_state_base.cpp
  src/robot_to_robot/kalman/results.cpp
  src/robot_to_robot/kalman/meta_state.cpp
  src/robot_to_robot/kalman/predictor.cpp
  src/robot_to_robot/kalman/updater_range.cpp
  src/robot_to_robot/kalman/updater_leader_pose.cpp
  src/robot_to_robot/particle/results.cpp
  src/robot_to_robot/particle/meta_state.cpp
  src/robot_to_robot/particle/predictor.cpp
  src/robot_to_robot/particle/updater_range.cpp
  src/robot_to_robot/particle/updater_leader_pose.cpp
  src/robot_to_world/lever_arm_compensation.cpp
  src/robot_to_world/meta_state_base.cpp
  src/robot_to_world/kalman/results.cpp
  src/robot_to_world/kalman/meta_state.cpp
  src/robot_to_world/kalman/predictor.cpp
  src/robot_to_world/kalman/updater_course.cpp
  src/robot_to_world/kalman/updater_range.cpp
  src/robot_to_world/kalman/updater_pose.cpp
  src/robot_to_world/kalman/updater_position.cpp
  src/robot_to_world/particle/predictor.cpp
  src/robot_to_world/particle/meta_state.cpp
  src/robot_to_world/particle/results.cpp
  src/robot_to_world/particle/updater_course.cpp
  src/robot_to_world/particle/updater_range.cpp
  src/robot_to_world/particle/updater_pose.cpp
  src/robot_to_world/particle/updater_position.cpp
  # src/robot_to_world_25D/kalman/R2WLocalisationKFResults25D.cpp
  # src/robot_to_world_25D/kalman/R2WLocalisationKFMetaState25D.cpp
  # src/robot_to_world_25D/kalman/R2WLocalisationKFPredictor25D.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${romea_core_common_INCLUDE_DIRS}
  ${romea_core_filtering_INCLUDE_DIRS}
  )

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall -Wextra -O3 -std=c++17)

target_link_libraries(${PROJECT_NAME} PUBLIC
  romea_core_common::romea_core_common)

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

option(BUILD_TESTING "BUILD WITH TESTS" ON)

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif(BUILD_TESTING)
