romea_add_gtest(test_r2w_meta_state test_meta_state.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_lever_arm_compensation test_lever_arm_compensation.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_attitude_updater test_attitude_updater.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_angular_speed_updater test_angular_speed_updater.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_linear_speed_updater test_linear_speed_updater.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_linear_speeds_updater test_linear_speeds_updater.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_twist_updater test_twist_updater.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_course_kf_updater kalman/test_course_updater.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_position_kf_updater kalman/test_position_updater.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_pose_kf_updater kalman/test_pose_updater.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_range_kf_updater kalman/test_range_updater.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_r2w_kf_initialisation_updaters kalman/test_initialisation_updaters.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_r2w_kf_predictor kalman/test_predictor.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_r2w_kf_results kalman/test_results.cpp LIBRARIES ${PROJECT_NAME})

add_subdirectory(particle)
