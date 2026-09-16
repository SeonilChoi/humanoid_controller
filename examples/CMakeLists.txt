add_executable(example_unitree src/hardware/motor/example_unitree.cpp)
target_link_libraries(example_unitree PRIVATE motor)

add_executable(example_cubemars src/hardware/motor/example_cubemars.cpp)
target_link_libraries(example_cubemars PRIVATE motor)

add_executable(example_dynamixel src/hardware/motor/example_dynamixel.cpp)
target_link_libraries(example_dynamixel PRIVATE motor)

add_executable(example_xsens src/hardware/sensor/example_xsens.cpp)
target_link_libraries(example_xsens PRIVATE sensor)

add_executable(example_dualsense src/hardware/joy/example_dualsense.cpp)
target_link_libraries(example_dualsense PRIVATE joy)