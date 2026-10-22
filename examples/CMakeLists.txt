add_executable(example_unitree src/hardware/motor/example_unitree.cpp)
target_link_libraries(example_unitree PRIVATE motor)

add_executable(example_cubemars src/hardware/motor/example_cubemars.cpp)
target_link_libraries(example_cubemars PRIVATE motor)

add_executable(example_dynamixel src/hardware/motor/example_dynamixel.cpp)
target_link_libraries(example_dynamixel PRIVATE motor)

add_executable(example_multi_drivers src/hardware/motor/example_multi_drivers.cpp)
target_link_libraries(example_multi_drivers PRIVATE motor)

add_executable(example_action src/hardware/motor/example_action.cpp)
target_link_libraries(example_action PRIVATE motor yaml-cpp)

add_executable(example_xsens src/hardware/sensor/example_xsens.cpp)
target_link_libraries(example_xsens PRIVATE sensor)

add_executable(example_dualsense src/hardware/joy/example_dualsense.cpp)
target_link_libraries(example_dualsense PRIVATE joy)
add_executable(example_sysid src/hardware/motor/example_sysid.cpp)
target_link_libraries(example_sysid PRIVATE motor yaml-cpp)
