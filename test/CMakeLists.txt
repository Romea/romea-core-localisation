find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME}_test_proprioceptive_updaters test_proprioceptive_updaters.cpp )
target_link_libraries(${PROJECT_NAME}_test_proprioceptive_updaters ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_proprioceptive_updaters PRIVATE -std=c++17)
add_test(test_proprioceptive_updaters ${PROJECT_NAME}_test_proprioceptive_updaters)

add_executable(${PROJECT_NAME}_test_conversions test_conversions.cpp )
target_link_libraries(${PROJECT_NAME}_test_conversions ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_conversions PRIVATE -std=c++17)
add_test(test_conversions ${PROJECT_NAME}_test_conversions)



