cmake_minimum_required(VERSION 3.10)

project(uut)
 
add_executable(${PROJECT_NAME} uut.cpp)

target_link_libraries(${PROJECT_NAME} monoDriveClient)
