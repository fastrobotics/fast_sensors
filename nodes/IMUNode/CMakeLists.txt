include_directories(
	driver/include/
	driver/NavXIMUDriver/include/
	${catkin_INCLUDE_DIRS}
)
add_subdirectory(driver)
add_subdirectory(node)
