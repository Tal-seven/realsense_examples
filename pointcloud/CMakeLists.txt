cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

project(pointcloud)
find_package(OpenGL REQUIRED)
add_executable(pointcloud pointcloud.cpp /home/talha/librealsense/examples/example.hpp)
set_property(TARGET pointcloud PROPERTY CXX_STANDARD 11)
target_link_libraries(pointcloud PRIVATE realsense2 glfw ${OPENGL_LIBRARIES})

