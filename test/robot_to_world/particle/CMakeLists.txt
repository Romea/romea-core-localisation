add_executable(${PROJECT_NAME}_test_r2w_pf_meta_state test_meta_state.cpp )
target_link_libraries(${PROJECT_NAME}_test_r2w_pf_meta_state ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_r2w_pf_meta_state PRIVATE -std=c++17)
add_test(test_r2w_pf_meta_state ${PROJECT_NAME}_test_r2w_pf_meta_state)

add_executable(${PROJECT_NAME}_test_r2w_pf_results test_results.cpp )
target_link_libraries(${PROJECT_NAME}_test_r2w_pf_results ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_r2w_pf_results PRIVATE -std=c++17)
add_test(test_r2w_pf_results ${PROJECT_NAME}_test_r2w_pf_results)

add_executable(${PROJECT_NAME}_test_r2w_pf_predictor test_predictor.cpp )
target_link_libraries(${PROJECT_NAME}_test_r2w_pf_predictor ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_r2w_pf_predictor PRIVATE -std=c++17)
add_test(test_r2w_pf_predictor ${PROJECT_NAME}_test_r2w_pf_predictor)
