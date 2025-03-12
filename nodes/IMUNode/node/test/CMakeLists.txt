catkin_add_gtest(test_IMUNodeProcess test_IMUNodeProcess.cpp)
target_link_libraries(test_IMUNodeProcess ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger imuNodeProcess)
add_dependencies(test_IMUNodeProcess eros_generate_messages_cpp)

add_rostest_gtest(test_IMUNode
    test_IMUNode.test
    test_IMUNode.cpp
)
add_dependencies(test_IMUNode imu_node)
target_link_libraries(test_IMUNode ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger)
