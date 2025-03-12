add_library(baseIMUDriver BaseIMUDriver.cpp)
target_link_libraries(baseIMUDriver ${catkin_LIBRARIES} logger prettyUtility convertUtility)