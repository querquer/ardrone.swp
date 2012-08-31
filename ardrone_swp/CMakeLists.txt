cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()

#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})



rosbuild_add_executable(bottom_follow_tag src/bottom_follow_tag.cpp src/Math.cpp src/Global.cpp src/Keyboard.cpp)

rosbuild_add_executable(front_follow_tag src/front_follow_tag.cpp src/Math.cpp src/Global.cpp src/Keyboard.cpp)

rosbuild_add_executable(follow_line src/follow_line.cpp src/Math.cpp src/Global.cpp src/Keyboard.cpp)

rosbuild_add_executable(Log src/Log.cpp)

#target_link_libraries(Log $(-lboost_filesystem) $(-lboost_system) )
rosbuild_link_boost(Log filesystem system)

#rosbuild_add_executable(start src/start.cpp src/Math.cpp src/Global.cpp src/Keyboard.cpp src/Delta.cpp)
