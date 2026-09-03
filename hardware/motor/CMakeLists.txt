add_library(motor 
    src/motor_manager.cpp
)

target_include_directories(motor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(motor PRIVATE yaml-cpp)

target_compile_features(motor PUBLIC cxx_std_17)