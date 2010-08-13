cmake_minimum_required (VERSION 2.4.6)
include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set (CMAKE_VERBOSE_MAKEFILE ON)
set (ROS_BUILD_TYPE Debug)
add_definitions (-Wall)

rosbuild_init ()
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set (LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg ()
#rosbuild_gensrv ()

rosbuild_add_library (di_controller_plugin src/di_controller_plugin.cpp)

rosbuild_add_executable (di_analysis src/di_analysis.cpp)
#target_link_libraries ()
