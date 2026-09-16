add_library(controller src/controllers/amp_controller.cpp)

target_include_directories(controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
    HINTS $ENV{HOME}/.local/include
)
find_library(ONNXRUNTIME_LIB onnxruntime
    HINTS $ENV{HOME}/.local/lib
)

target_include_directories(controller PUBLIC ${ONNXRUNTIME_INCLUDE_DIR})

target_link_libraries(controller PUBLIC yaml-cpp ${ONNXRUNTIME_LIB})

target_compile_features(controller PUBLIC cxx_std_17)

add_library(robot src/robots/ARTI_H1.cpp)

target_include_directories(robot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(robot PUBLIC
    motor
    sensor
    joy
    controller
    yaml-cpp
    Eigen3::Eigen
    pinocchio::pinocchio
)