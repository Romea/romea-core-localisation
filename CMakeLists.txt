cmake_minimum_required(VERSION 3.5)

project(
  "romea_core_localisation"
  VERSION 1.0.0
  #  DESCRIPTION
  #  HOMEPAGE_URL
  LANGUAGES CXX)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(romea_core_common REQUIRED)
find_package(romea_core_filtering REQUIRED)

include(GNUInstallDirs)

add_library(${PROJECT_NAME} SHARED
  src/fsm_state.cpp
  src/updater_base.cpp
  src/updater_exteroceptive.cpp
  src/updater_proprioceptive.cpp
  src/updater_trigger_mode.cpp
  src/robot_to_human/meta_state_base.cpp
  src/robot_to_human/kalman/meta_state.cpp
  src/robot_to_human/kalman/predictor.cpp
  src/robot_to_human/kalman/updater_range.cpp
  src/robot_to_human/kalman/updater_leader_position.cpp
  src/robot_to_robot/meta_state_base.cpp
  src/robot_to_robot/kalman/meta_state.cpp
  src/robot_to_robot/kalman/predictor.cpp
  src/robot_to_robot/kalman/updater_range.cpp
  src/robot_to_robot/kalman/updater_leader_pose.cpp
  src/robot_to_robot/particle/meta_state.cpp
  src/robot_to_robot/particle/predictor.cpp
  src/robot_to_robot/particle/updater_range.cpp
  src/robot_to_robot/particle/updater_leader_pose.cpp
  src/robot_to_world/lever_arm_compensation.cpp
  src/robot_to_world/meta_state_base.cpp
  src/robot_to_world/kalman/meta_state.cpp
  src/robot_to_world/kalman/predictor.cpp
  src/robot_to_world/kalman/updater_course.cpp
  src/robot_to_world/kalman/updater_range.cpp
  src/robot_to_world/kalman/updater_pose.cpp
  src/robot_to_world/kalman/updater_position.cpp
  src/robot_to_world/particle/predictor.cpp
  src/robot_to_world/particle/meta_state.cpp
  src/robot_to_world/particle/updater_course.cpp
  src/robot_to_world/particle/updater_range.cpp
  src/robot_to_world/particle/updater_pose.cpp
  src/robot_to_world/particle/updater_position.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall -Wextra -O3)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  romea_core_common::romea_core_common
  romea_core_filtering::romea_core_filtering)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_Targets
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
  EXPORT ${PROJECT_NAME}_Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()
