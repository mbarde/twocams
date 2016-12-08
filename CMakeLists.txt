cmake_minimum_required(VERSION 2.8.12)

project(twocams)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(detection detection.cpp)

add_executable(twocams twocams.cpp)
target_link_libraries(twocams ${OpenCV_LIBS} detection)
