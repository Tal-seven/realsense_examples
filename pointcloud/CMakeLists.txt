cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

project(pointcloud)
find_package(OpenGL REQUIRED)
add_executable(pointcloud pointcloud.cpp /home/talha/librealsense/examples/example.hpp)
set_property(TARGET pointcloud PROPERTY CXX_STANDARD 11)
target_link_libraries(pointcloud PRIVATE realsense2 glfw ${OPENGL_LIBRARIES})

#[[find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
FILE(GLOB Sources ${CMAKE_SOURCE_DIR}/imgui/*.cpp)
add_executable(pointcloudGit pointcloudGit.cpp /home/talha/librealsense/examples/example.hpp ${Sources})
set_property(TARGET pointcloudGit PROPERTY CXX_STANDARD 11)
message(INFO,"linking opencv libraries ${OPENCV_LIBS}")
target_link_libraries(pointcloudGit PRIVATE realsense2 glfw ${OPENGL_LIBRARIES} ${PCL_LIBRARIES} ${OPENCV_LIBS})
target_include_directories(pointcloudGit PRIVATE ${PCL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/imgui/)
target_compile_definitions(pointcloudGit PRIVATE ${PCL_DEFINITIONS})]]

