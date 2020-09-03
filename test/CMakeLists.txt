catkin_add_gtest(${PROJECT_NAME}_test_proprioceptive_updaters test_proprioceptive_updaters.cpp )
target_link_libraries(${PROJECT_NAME}_test_proprioceptive_updaters ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}_test_conversions test_conversions.cpp )
target_link_libraries(${PROJECT_NAME}_test_conversions ${PROJECT_NAME})


