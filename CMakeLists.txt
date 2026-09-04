cmake_minimum_required(VERSION 3.16)
project(humanoid_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)

add_subdirectory(hardware)

add_executable(motor_manager examples/motor_manager_test.cpp)

target_link_libraries(motor_manager PRIVATE motor)