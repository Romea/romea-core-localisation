romea_add_gtest(test_r2h_meta_state test_meta_state.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_r2h_leader_position_updater test_leader_position_updater.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_r2h_range_updater test_range_updater.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_r2h_kf_predictor test_kalman_predictor.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_r2h_kf_results test_kalman_results.cpp LIBRARIES ${PROJECT_NAME})
