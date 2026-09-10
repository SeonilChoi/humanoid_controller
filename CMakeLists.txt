cmake_minimum_required(VERSION 3.16)
project(humanoid_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(PREPEND CMAKE_PREFIX_PATH "/opt/openrobots")

find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_subdirectory(hardware)
add_subdirectory(robot)

add_executable(motor_manager examples/motor_manager_test.cpp)
target_link_libraries(motor_manager PRIVATE motor)

add_executable(sensor_manager examples/sensor_manager_test.cpp)
target_link_libraries(sensor_manager PRIVATE sensor)

add_executable(robot_test examples/robot_test.cpp)
target_link_libraries(robot_test PRIVATE olaf)

add_executable(joy_test examples/joy_test.cpp)
target_link_libraries(joy_test PRIVATE joy)