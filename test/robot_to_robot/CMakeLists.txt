add_executable(${PROJECT_NAME}_test_r2r_meta_state test_meta_state.cpp )
target_link_libraries(${PROJECT_NAME}_test_r2r_meta_state ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_r2r_meta_state PRIVATE -std=c++17)
add_test(test_r2r_meta_state ${PROJECT_NAME}_test_r2r_meta_state)

add_executable(${PROJECT_NAME}_test_r2r_leader_twist_updater test_leader_twist_updater.cpp )
target_link_libraries(${PROJECT_NAME}_test_r2r_leader_twist_updater ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_r2r_leader_twist_updater PRIVATE -std=c++17)
add_test(test_r2r_leader_twist_updater ${PROJECT_NAME}_test_r2r_leader_twist_updater)

add_executable(${PROJECT_NAME}_test_r2r_leader_pose_updater kalman/test_leader_pose_updater.cpp )
target_link_libraries(${PROJECT_NAME}_test_r2r_leader_pose_updater ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_r2r_leader_pose_updater PRIVATE -std=c++17)
add_test(test_r2r_leader_pose_updater ${PROJECT_NAME}_test_r2r_leader_pose_updater)

add_executable(${PROJECT_NAME}_test_r2r_range_updater kalman/test_range_updater.cpp )
target_link_libraries(${PROJECT_NAME}_test_r2r_range_updater ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_r2r_range_updater PRIVATE -std=c++17)
add_test(test_r2r_range_updater ${PROJECT_NAME}_test_r2r_range_updater)

add_executable(${PROJECT_NAME}_test_r2r_kf_predictor kalman/test_predictor.cpp )
target_link_libraries(${PROJECT_NAME}_test_r2r_kf_predictor ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_r2r_kf_predictor PRIVATE -std=c++17)
add_test(test_r2r_kf_predictor ${PROJECT_NAME}_test_r2r_kf_predictor)

add_executable(${PROJECT_NAME}_test_r2r_kf_results kalman/test_results.cpp )
target_link_libraries(${PROJECT_NAME}_test_r2r_kf_results ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_r2r_kf_results PRIVATE -std=c++17)
add_test(test_r2r_kf_results ${PROJECT_NAME}_test_r2r_kf_results)

add_subdirectory(particle)
