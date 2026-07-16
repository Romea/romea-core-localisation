romea_add_gtest(test_r2r_meta_state test_meta_state.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_r2r_leader_twist_updater test_leader_twist_updater.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_r2r_leader_pose_updater kalman/test_leader_pose_updater.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_r2r_range_updater kalman/test_range_updater.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_r2r_kf_predictor kalman/test_predictor.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_r2r_kf_results kalman/test_results.cpp LIBRARIES ${PROJECT_NAME})

add_subdirectory(particle)
