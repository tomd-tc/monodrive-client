cmake_minimum_required(VERSION 3.10)

project(replay)
 
add_executable(${PROJECT_NAME} replay.cpp)

target_link_libraries(${PROJECT_NAME} monodrive)
