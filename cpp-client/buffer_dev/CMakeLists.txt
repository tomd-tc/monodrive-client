cmake_minimum_required(VERSION 3.10)

project(buffer_dev)
 
add_executable(${PROJECT_NAME} buffer_dev.cpp)
add_executable(state_sensor state_sensor.cpp)

find_package(OpenCV REQUIRED PATHS "C:/local/opencv/build")

target_link_libraries(${PROJECT_NAME} monoDriveClient)

find_package(OpenCV REQUIRED PATHS "C:/local/opencv/build" "C:/opencv/build")
include_directories(${OpenCV_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

target_link_libraries(state_sensor monoDriveClient)
target_include_directories(state_sensor PUBLIC ${OpenCV_INCLUDE_DIRS})
link_libraries(state_sensor ${OpenCV_LIBRARIES})
target_link_libraries(state_sensor ${OpenCV_LIBRARIES})