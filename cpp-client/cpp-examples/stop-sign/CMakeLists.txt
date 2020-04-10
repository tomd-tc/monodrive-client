cmake_minimum_required(VERSION 3.10)

project(stop-sign)
 
add_executable(stop-sign stop-sign.cpp)

target_link_libraries(stop-sign monoDriveClient)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message(${EIGEN3_INCLUDE_DIR})
target_link_libraries(stop-sign Eigen3::Eigen)