cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

project(pointCloudLibrary)
find_package(OpenGL REQUIRED)
find_package(PCL 1.8 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
FILE(GLOB Sources ${CMAKE_SOURCE_DIR}/imgui/*.cpp)

add_executable(pcl pcl.cpp /home/talha/librealsense/examples/example.hpp ${Sources})
set_property(TARGET pcl PROPERTY CXX_STANDARD 11)
target_link_libraries(pcl PRIVATE realsense2 glfw ${OPENGL_LIBRARIES} ${PCL_LIBRARIES})
target_include_directories(pcl PRIVATE ${PCL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/imgui/)
target_compile_definitions(pcl PRIVATE ${PCL_DEFINITIONS})

add_executable(rspcl rs-pcl.cpp /home/talha/librealsense/examples/example.hpp ${Sources})
set_property(TARGET rspcl PROPERTY CXX_STANDARD 11)
target_link_libraries(rspcl PRIVATE realsense2 glfw ${OPENGL_LIBRARIES} ${PCL_LIBRARIES})
target_include_directories(rspcl PRIVATE ${PCL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/imgui/)
target_compile_definitions(rspcl PRIVATE ${PCL_DEFINITIONS})
