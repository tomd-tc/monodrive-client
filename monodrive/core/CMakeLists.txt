cmake_minimum_required(VERSION 3.10)

project(monodrive)

set(object_lib_name monodrive_object)
set(shared_lib_name ${PROJECT_NAME})
set(static_lib_name monodrive_static)

# collect headers and source
file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "src/*.h" "src/*.hpp")

# define object library
add_library(${object_lib_name} OBJECT ${SOURCES})
set_property(TARGET ${object_lib_name} PROPERTY POSITION_INDEPENDENT_CODE ON)

# set c++ 11 standard
set_target_properties(
	${object_lib_name}
    PROPERTIES
        CXX_STANDARD 11
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)

# bring src directory into the project
include_directories(src)
target_include_directories(${object_lib_name} PUBLIC src)

# include boost
IF(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.12.0")
	cmake_policy(SET CMP0074 NEW)
ENDIF(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.12.0")
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_LIBS OFF)
find_package(Boost 1.65 REQUIRED COMPONENTS system)
if(MSVC)
	target_compile_definitions(${object_lib_name} PUBLIC -DBOOST_ALL_NO_LIB)
	target_compile_definitions(${object_lib_name} PUBLIC -DBOOST_PROGRAM_OPTIONS_DYN_LINK)
endif(MSVC)

include_directories(${Boost_INCLUDE_DIRS})
target_include_directories(${object_lib_name} PUBLIC ${Boost_INCLUDE_DIRS})
link_libraries(${object_lib_name} ${Boost_LIBRARIES})
target_link_libraries(${object_lib_name} ${Boost_LIBRARIES})

find_library(LIBRT rt)
find_package(Threads)

if(LIBRT)
	target_link_libraries(${object_lib_name} ${LIBRT})
endif(LIBRT)
target_link_libraries (${object_lib_name} ${CMAKE_THREAD_LIBS_INIT})

# create static library
add_library(${static_lib_name} STATIC $<TARGET_OBJECTS:${object_lib_name}>)

# create shared library
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
add_library(${shared_lib_name} SHARED $<TARGET_OBJECTS:${object_lib_name}>)

# installation
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX /usr/local/monodrive/client CACHE PATH "monoDrive client install prefix" FORCE)
endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

install(TARGETS ${shared_lib_name} DESTINATION lib)
install(FILES ${HEADERS} DESTINATION include)
