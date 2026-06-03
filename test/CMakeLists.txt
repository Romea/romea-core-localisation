find_package(GTest REQUIRED)

add_subdirectory(robot_to_world)
add_subdirectory(robot_to_robot)
add_subdirectory(robot_to_human)

#add_executable(${PROJECT_NAME}_test_proprioceptive_updaters test_proprioceptive_updaters.cpp )
#target_link_libraries(${PROJECT_NAME}_test_proprioceptive_updaters ${PROJECT_NAME} GTest::GTest GTest::Main)
#target_compile_options(${PROJECT_NAME}_test_proprioceptive_updaters PRIVATE -std=c++17)
#add_test(test_proprioceptive_updaters ${PROJECT_NAME}_test_proprioceptive_updaters)

add_executable(${PROJECT_NAME}_test_fsm_state_conversions test_fsm_state_conversions.cpp )
target_link_libraries(${PROJECT_NAME}_test_fsm_state_conversions ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_fsm_state_conversions PRIVATE -std=c++17)
add_test(test_fsm_state_conversions ${PROJECT_NAME}_test_fsm_state_conversions)

add_executable(${PROJECT_NAME}_test_update_monitoring test_update_monitoring.cpp )
target_link_libraries(${PROJECT_NAME}_test_update_monitoring ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_update_monitoring PRIVATE -std=c++17)
add_test(test_update_monitoring ${PROJECT_NAME}_test_update_monitoring)

add_executable(${PROJECT_NAME}_test_traits test_traits.cpp )
target_link_libraries(${PROJECT_NAME}_test_traits ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_traits PRIVATE -std=c++17)
add_test(test_traits ${PROJECT_NAME}_test_traits)
