add_library(imuNodeProcess IMUNodeProcess.cpp)
target_link_libraries(imuNodeProcess ${catkin_LIBRARIES} baseNodeProcess navXIMUDriver )

add_executable(imu_node IMUNode.cpp) 
target_link_libraries(imu_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} imuNodeProcess baseNode   )
add_dependencies(imu_node ${eros_EXPORTED_TARGETS})
#add_subdirectory(test)
