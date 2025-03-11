include_directories(
    driver/include/
    driver/NavXIMUDriver/include/
    ${catkin_INCLUDE_DIRS}
)
add_subdirectory(src)
#add_subdirectory(NavXIMUDriver)