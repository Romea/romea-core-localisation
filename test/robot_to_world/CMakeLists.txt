add_executable(${PROJECT_NAME}_test_level_arm_compensation test_level_arm_compensation.cpp )
target_link_libraries(${PROJECT_NAME}_test_level_arm_compensation ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_level_arm_compensation PRIVATE -std=c++17)
add_test(test_level_arm_compensation ${PROJECT_NAME}_test_level_arm_compensation)

add_executable(${PROJECT_NAME}_test_attitude_updater test_attitude_updater.cpp )
target_link_libraries(${PROJECT_NAME}_test_attitude_updater ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_attitude_updater PRIVATE -std=c++17)
add_test(test_attitude_updater ${PROJECT_NAME}_test_attitude_updater)

add_executable(${PROJECT_NAME}_test_angular_speed_updater test_angular_speed_updater.cpp )
target_link_libraries(${PROJECT_NAME}_test_angular_speed_updater ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_angular_speed_updater PRIVATE -std=c++17)
add_test(test_angular_speed_updater ${PROJECT_NAME}_test_angular_speed_updater)

add_executable(${PROJECT_NAME}_test_linear_speed_updater test_linear_speed_updater.cpp )
target_link_libraries(${PROJECT_NAME}_test_linear_speed_updater ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_linear_speed_updater PRIVATE -std=c++17)
add_test(test_linear_speed_updater ${PROJECT_NAME}_test_linear_speed_updater)

add_executable(${PROJECT_NAME}_test_linear_speeds_updater test_linear_speeds_updater.cpp )
target_link_libraries(${PROJECT_NAME}_test_linear_speeds_updater ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_linear_speeds_updater PRIVATE -std=c++17)
add_test(test_linear_speeds_updater ${PROJECT_NAME}_test_linear_speeds_updater)

add_executable(${PROJECT_NAME}_test_twist_updater test_twist_updater.cpp )
target_link_libraries(${PROJECT_NAME}_test_twist_updater ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_twist_updater PRIVATE -std=c++17)
add_test(test_twist_updater ${PROJECT_NAME}_test_twist_updater)
