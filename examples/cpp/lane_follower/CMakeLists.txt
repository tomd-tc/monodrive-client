cmake_minimum_required(VERSION 3.10)

project(lane_follower)
 
add_executable(${PROJECT_NAME} lane_follower.cpp)

target_link_libraries(${PROJECT_NAME} monodrive)

include_directories(${EIGEN3_INCLUDE_DIR})