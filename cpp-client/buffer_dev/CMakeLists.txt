cmake_minimum_required(VERSION 3.10)

project(camera_sensor)
 
add_executable(${PROJECT_NAME} camera_sensor.cpp)
add_executable(state_sensor state_sensor.cpp)
add_executable(imu_sensor imu_sensor.cpp)
add_executable(gps_sensor gps_sensor.cpp)
add_executable(radar_sensor radar_sensor.cpp)

find_package(OpenCV REQUIRED PATHS "C:/local/opencv/build")

target_link_libraries(${PROJECT_NAME} monoDriveClient)
target_link_libraries(state_sensor monoDriveClient)
target_link_libraries(imu_sensor monoDriveClient)
target_link_libraries(gps_sensor monoDriveClient)
target_link_libraries(radar_sensor monoDriveClient)

find_package(OpenCV REQUIRED PATHS "C:/local/opencv/build" "C:/opencv/build")
include_directories(${OpenCV_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
