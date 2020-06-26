cmake_minimum_required(VERSION 3.10)

project(replay)
 
add_executable(${PROJECT_NAME} replay.cpp)

target_link_libraries(${PROJECT_NAME} monodrive)

# copy dll to runtime directory
if(MSVC)
  add_custom_command(TARGET replay POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:monodrive>
      $<TARGET_FILE_DIR:replay>
  )
endif(MSVC)
