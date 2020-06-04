cmake_minimum_required(VERSION 3.10)

project(replay_step)
 
add_executable(${PROJECT_NAME} replay_step.cpp)

target_link_libraries(${PROJECT_NAME} monodrive)
