add_library(NavXIMUDriver NavXIMUDriver.cpp)
target_link_libraries(NavXIMUDriver ${catkin_LIBRARIES} baseIMUDriver)
#add_subdirectory(test)