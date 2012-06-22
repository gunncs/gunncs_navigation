cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init ()
rosbuild_add_boost_directories ()

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories (${CMAKE_CURRENT_BINARY_DIR}) 
include_directories (src)

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories(${wxWidgets_INCLUDE_DIRS})

find_package (VTK REQUIRED)
include (${VTK_USE_FILE})

#include library for pcl, corrects bug in fuerte.
#https://code.ros.org/trac/ros-pkg/ticket/5419
#include_directories("/usr/include/vtk-5.8")
#include("/opt/ros/fuerte/share/pcl-1.5/PCLConfigVersion.cmake")
#include("/opt/ros/fuerte/share/pcl-1.5/PCLConfig.cmake")

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization #  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries #set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()



#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#find_package(PCL 1.2 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_executable(distanceDiag src/distanceDiag.cpp)
#rosbuild_add_executable(cannyGroundDetect src/cannyGroundDetect.cpp)
rosbuild_add_executable(pclDemo src/pclDemo.cpp)
target_link_libraries(pclDemo libvtkCommon.so libvtkFiltering.so pcl_visualization gomp)


rosbuild_add_executable(pclGroundDetect src/pclGroundDetect.cpp)

#rosbuild_add_executable(viewer src/viewer.cpp)
#rosbuild_add_executable(segmentation_plane src/segmentation_plane.cpp)
#rosbuild_add_executable(extract_indicies src/extract_indicies.cpp)

rosbuild_add_executable(pidDistanceDemo src/pidDistanceDemo.cpp)
rosbuild_add_executable(derivField src/derivField.cpp)

#rosbuild_add_executable(test_shapes src/test_shapes.cpp)

#rosbuild_add_executable(ransac src/ransac.cpp)
#target_link_libraries(ransac libvtkCommon.so libvtkFiltering.so pcl_visualization gomp ${PCL_LIBRARIES})
# needed for pcl


