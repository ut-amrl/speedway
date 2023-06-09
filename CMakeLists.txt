project(speedway)
cmake_minimum_required(VERSION 3.16)

message(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

set(CMAKE_EXPORT_COMPILE_COMMANDS on)
set(CMAKE_CXX_FLAGS "-std=c++14 -Wall -Wextra")

if(${CMAKE_BUILD_TYPE} MATCHES "Release")
  message(STATUS "Additional Flags for Release mode")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O2 -DNDEBUG")
elseif(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  message(STATUS "Additional Flags for Debug mode")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
endif()

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
set(ROS_BUILD_STATIC_LIBS true)
set(ROS_BUILD_SHARED_LIBS false)

message(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(libs roslib roscpp rosbag glog gflags amrl_shared_lib boost_system lua5.1
    pthread)

# builds libspeedway
add_subdirectory(src)

include_directories(src/shared)
include_directories(src/config_reader/include)
include_directories(src)

rosbuild_add_executable(race
                        src/race/race_main.cpp
                        src/visualization/visualization.cc)
target_link_libraries(race speedway ${libs})
