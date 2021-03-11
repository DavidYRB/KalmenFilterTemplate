project(KalmanFilterTemplate)

cmake_minimum_required (VERSION 3.1)

add_definitions(-std=c++11)


set(sources src/main.cpp src/kalmanFilter.cpp src/rangeSensor.cpp src/sensor.hpp src/filter.hpp src/constants.hpp)

add_executable(KalmanFilterTemplate ${sources})

target_include_directories (KalmanFilterTemplate PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/Eigen)
