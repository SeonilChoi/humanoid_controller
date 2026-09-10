add_library(joy 
    src/joy_handler.cpp
    src/playstation/dualsense.cpp
)

target_include_directories(joy
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(joy
    PUBLIC
        yaml-cpp
        Threads::Threads
)

target_compile_features(joy PUBLIC cxx_std_17)