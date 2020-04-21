cmake_minimum_required(VERSION 3.10)

project(monoDriveClient)

file(GLOB SOURCES "src/*.cpp")

add_library(${PROJECT_NAME} ${SOURCES})
 
#Bring the headers, into the project
include_directories(src)
target_include_directories(${PROJECT_NAME} PUBLIC src)

#include boost
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_LIBS OFF)
find_package(Boost 1.65 REQUIRED COMPONENTS system)
if(MSVC)
	target_compile_definitions(${PROJECT_NAME} PUBLIC -DBOOST_ALL_NO_LIB)
	target_compile_definitions(${PROJECT_NAME} PUBLIC -DBOOST_PROGRAM_OPTIONS_DYN_LINK)
else()
	# not required for msvc
	target_link_libraries(${PROJECT_NAME} -lstdc++fs)
endif()


include_directories(${Boost_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${Boost_INCLUDE_DIRS})
link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

find_library(LIBRT rt)
find_package(Threads)

if(LIBRT)
	target_link_libraries(${PROJECT_NAME} ${LIBRT})
endif(LIBRT)
target_link_libraries (${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
