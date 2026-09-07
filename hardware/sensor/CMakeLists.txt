find_package(yaml-cpp REQUIRED)

add_custom_target(xspublic_build
    COMMAND make
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic
)

add_library(sensor
    src/sensor_manager.cpp
    src/imu/xsens_mti.cpp
)

add_dependencies(sensor xspublic_build)

target_include_directories(sensor
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic
)

target_link_libraries(sensor
    PUBLIC
        yaml-cpp

        ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xscontroller/libxscontroller.a
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xscommon/libxscommon.a
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xstypes/libxstypes.a

        pthread
        dl
)