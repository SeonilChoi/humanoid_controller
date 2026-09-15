add_executable(motor_manager motor_manager_test.cpp)
target_link_libraries(motor_manager PRIVATE motor)

add_executable(sensor_manager sensor_manager_test.cpp)
target_link_libraries(sensor_manager PRIVATE sensor)

add_executable(joy_test joy_test.cpp)
target_link_libraries(joy_test PRIVATE joy)

add_executable(robot_test robot_test.cpp)
target_link_libraries(robot_test PRIVATE olaf)