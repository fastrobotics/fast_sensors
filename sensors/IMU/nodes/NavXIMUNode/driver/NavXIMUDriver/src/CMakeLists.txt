add_library(NavXIMUDriver NavXIMUDriver.cpp)
target_link_libraries(NavXIMUDriver ${catkin_LIBRARIES} logger prettyUtility convertUtility)
#add_subdirectory(test)