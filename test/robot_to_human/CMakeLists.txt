add_executable(${PROJECT_NAME}_test_r2h_meta_state test_meta_state.cpp )
target_link_libraries(${PROJECT_NAME}_test_r2h_meta_state ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_r2h_meta_state PRIVATE -std=c++17)
add_test(test_r2h_meta_state ${PROJECT_NAME}_test_r2h_meta_state)

add_executable(${PROJECT_NAME}_test_r2h_leader_position_updater test_leader_position_updater.cpp )
target_link_libraries(${PROJECT_NAME}_test_r2h_leader_position_updater ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_r2h_leader_position_updater PRIVATE -std=c++17)
add_test(test_r2h_leader_position_updater ${PROJECT_NAME}_test_r2h_leader_position_updater)

add_executable(${PROJECT_NAME}_test_r2h_range_updater test_range_updater.cpp )
target_link_libraries(${PROJECT_NAME}_test_r2h_range_updater ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_r2h_range_updater PRIVATE -std=c++17)
add_test(test_r2h_range_updater ${PROJECT_NAME}_test_r2h_range_updater)

add_executable(${PROJECT_NAME}_test_r2h_kf_predictor test_kalman_predictor.cpp )
target_link_libraries(${PROJECT_NAME}_test_r2h_kf_predictor ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_r2h_kf_predictor PRIVATE -std=c++17)
add_test(test_r2h_kf_predictor ${PROJECT_NAME}_test_r2h_kf_predictor)

add_executable(${PROJECT_NAME}_test_r2h_kf_results test_kalman_results.cpp )
target_link_libraries(${PROJECT_NAME}_test_r2h_kf_results ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_r2h_kf_results PRIVATE -std=c++17)
add_test(test_r2h_kf_results ${PROJECT_NAME}_test_r2h_kf_results)
