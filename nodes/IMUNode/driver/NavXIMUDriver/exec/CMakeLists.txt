add_executable(exec_navvximu_driver exec_NavXIMUDriver.cpp) 
target_link_libraries(exec_navvximu_driver ${catkin_LIBRARIES} navXIMUDriver )
install(TARGETS exec_navvximu_driver
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
