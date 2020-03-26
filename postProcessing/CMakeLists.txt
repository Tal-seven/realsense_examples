cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

project(postProcessing)
find_package(OpenGL REQUIRED)
FILE(GLOB Sources ${CMAKE_SOURCE_DIR}/imgui/*.cpp)
message(INFO,"${Sources}")
add_executable(post-processing postProcessing.cpp /home/talha/librealsense/examples/example.hpp ${Sources})

set_property(TARGET post-processing PROPERTY CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
target_include_directories(post-processing PRIVATE 
													${CMAKE_SOURCE_DIR}/imgui/
													/home/talha/librealsense/third-party/tclap/include)
target_link_libraries(post-processing PRIVATE realsense2 glfw ${OPENGL_LIBRARIES})

