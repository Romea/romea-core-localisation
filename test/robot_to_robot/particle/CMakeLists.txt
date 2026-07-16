romea_add_gtest(test_r2r_pf_meta_state test_meta_state.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_r2r_pf_results test_results.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_r2r_pf_predictor test_predictor.cpp LIBRARIES ${PROJECT_NAME})
