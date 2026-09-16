add_executable(example_dynamixel src/hardware/motor/example_dynamixel.cpp)
target_link_libraries(example_dynamixel PRIVATE motor)

add_executable(example_unitree src/hardware/motor/example_unitree.cpp)
target_link_libraries(example_unitree PRIVATE motor)

add_executable(sensor_manager src/sensor_manager_test.cpp)
target_link_libraries(sensor_manager PRIVATE sensor)

add_executable(joy_test src/joy_test.cpp)
target_link_libraries(joy_test PRIVATE joy)

add_executable(robot_test src/robot_test.cpp)
target_link_libraries(robot_test PRIVATE olaf)

add_executable(controller_test src/controller_test.cpp)
target_link_libraries(controller_test PRIVATE controller)