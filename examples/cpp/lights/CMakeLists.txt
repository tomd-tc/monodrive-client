cmake_minimum_required(VERSION 3.10)

project(lights)
 
add_executable(${PROJECT_NAME} lights.cpp)

target_link_libraries(${PROJECT_NAME} monodrive)

# copy dll to runtime directory
if(MSVC)
  add_custom_command(TARGET lights POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:monodrive>
      $<TARGET_FILE_DIR:lights>
  )
endif(MSVC)
