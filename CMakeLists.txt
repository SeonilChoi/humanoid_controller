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
add_subdirectory(examples)