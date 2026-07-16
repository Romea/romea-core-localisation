find_package(GTest REQUIRED)
include(romea_add_gtest)

add_subdirectory(robot_to_world)
add_subdirectory(robot_to_robot)
add_subdirectory(robot_to_human)

#add_executable(${PROJECT_NAME}_test_proprioceptive_updaters test_proprioceptive_updaters.cpp )
#target_link_libraries(${PROJECT_NAME}_test_proprioceptive_updaters ${PROJECT_NAME} GTest::GTest GTest::Main)
#target_compile_options(${PROJECT_NAME}_test_proprioceptive_updaters PRIVATE -std=c++17)
#add_test(test_proprioceptive_updaters ${PROJECT_NAME}_test_proprioceptive_updaters)

romea_add_gtest(test_fsm_state_conversions test_fsm_state_conversions.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_update_monitoring test_update_monitoring.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_traits test_traits.cpp LIBRARIES ${PROJECT_NAME})
