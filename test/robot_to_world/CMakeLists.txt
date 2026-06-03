add_executable(${PROJECT_NAME}_test_r2w_meta_state test_meta_state.cpp )
target_link_libraries(${PROJECT_NAME}_test_r2w_meta_state ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_r2w_meta_state PRIVATE -std=c++17)
add_test(test_r2w_meta_state ${PROJECT_NAME}_test_r2w_meta_state)

add_executable(${PROJECT_NAME}_test_lever_arm_compensation test_lever_arm_compensation.cpp )
target_link_libraries(${PROJECT_NAME}_test_lever_arm_compensation ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_lever_arm_compensation PRIVATE -std=c++17)
add_test(test_lever_arm_compensation ${PROJECT_NAME}_test_lever_arm_compensation)

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

add_executable(${PROJECT_NAME}_test_course_kf_updater kalman/test_course_updater.cpp )
target_link_libraries(${PROJECT_NAME}_test_course_kf_updater ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_course_kf_updater PRIVATE -std=c++17)
add_test(test_course_kf_updater ${PROJECT_NAME}_test_course_kf_updater)

add_executable(${PROJECT_NAME}_test_position_kf_updater kalman/test_position_updater.cpp )
target_link_libraries(${PROJECT_NAME}_test_position_kf_updater ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_position_kf_updater PRIVATE -std=c++17)
add_test(test_position_kf_updater ${PROJECT_NAME}_test_position_kf_updater)

add_executable(${PROJECT_NAME}_test_pose_kf_updater kalman/test_pose_updater.cpp )
target_link_libraries(${PROJECT_NAME}_test_pose_kf_updater ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_pose_kf_updater PRIVATE -std=c++17)
add_test(test_pose_kf_updater ${PROJECT_NAME}_test_pose_kf_updater)

add_executable(${PROJECT_NAME}_test_range_kf_updater kalman/test_range_updater.cpp )
target_link_libraries(${PROJECT_NAME}_test_range_kf_updater ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_range_kf_updater PRIVATE -std=c++17)
add_test(test_range_kf_updater ${PROJECT_NAME}_test_range_kf_updater)

add_executable(${PROJECT_NAME}_test_r2w_kf_predictor kalman/test_predictor.cpp )
target_link_libraries(${PROJECT_NAME}_test_r2w_kf_predictor ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_r2w_kf_predictor PRIVATE -std=c++17)
add_test(test_r2w_kf_predictor ${PROJECT_NAME}_test_r2w_kf_predictor)

add_executable(${PROJECT_NAME}_test_r2w_kf_results kalman/test_results.cpp )
target_link_libraries(${PROJECT_NAME}_test_r2w_kf_results ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_r2w_kf_results PRIVATE -std=c++17)
add_test(test_r2w_kf_results ${PROJECT_NAME}_test_r2w_kf_results)

add_subdirectory(particle)
