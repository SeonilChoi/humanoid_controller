add_executable(motor_manager src/motor_manager_test.cpp)
target_link_libraries(motor_manager PRIVATE motor)

add_executable(sensor_manager src/sensor_manager_test.cpp)
target_link_libraries(sensor_manager PRIVATE sensor)

add_executable(joy_test src/joy_test.cpp)
target_link_libraries(joy_test PRIVATE joy)

add_executable(robot_test src/robot_test.cpp)
target_link_libraries(robot_test PRIVATE olaf)

add_executable(controller_test src/controller_test.cpp)
target_link_libraries(controller_test PRIVATE controller)