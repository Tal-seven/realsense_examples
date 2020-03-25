cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

project(hellorealsense)
find_package(realsense2 REQUIRED)
add_executable(realsense hello_realsense.cpp)
set_property(TARGET realsense PROPERTY CXX_STANDARD 11)
target_link_libraries(realsense ${realsense2_LIBRARY})
