add_library(navXIMUPacketParser NavXIMUPacketParser.cpp)

add_library(navXIMUDriver NavXIMUDriver.cpp)
target_link_libraries(navXIMUDriver ${catkin_LIBRARIES} baseIMUDriver navXIMUPacketParser)
add_subdirectory(test)