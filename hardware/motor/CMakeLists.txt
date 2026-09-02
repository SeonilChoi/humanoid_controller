add_library(motor src/motor_manager.cpp)

target_include_directories(motor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)