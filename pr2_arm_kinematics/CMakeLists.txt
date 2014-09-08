# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_arm_kinematics)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread)

# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp angles tf_conversions urdf geometry_msgs kdl_parser moveit_msgs moveit_core pluginlib eigen)


include_directories(include ${catkin_INCLUDE_DIRS})

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#add_message_files(

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})




add_library(pr2_arm_kinematics_lib
  src/pr2_arm_ik.cpp
  src/pr2_arm_ik_solver.cpp
  src/pr2_arm_kinematics_utils.cpp
  src/pr2_arm_kinematics.cpp
  src/pr2_arm_kinematics_plugin.cpp
)




add_executable(pr2_arm_kinematics_node src/main.cpp)
target_link_libraries(pr2_arm_kinematics_node pr2_arm_kinematics_lib ${catkin_LIBRARIES})
target_link_libraries(pr2_arm_kinematics_lib ${catkin_LIBRARIES})
 
#add_executable(ik_with_cost_test test/test_ik_with_cost.cpp)
#target_link_libraries(ik_with_cost_test pr2_arm_ik)

#add_executable(pr2_ik_regression_test test/test_ik_regress.cpp)
#rosbuild_declare_test(pr2_ik_regression_test)
#target_link_libraries(pr2_ik_regression_test pr2_arm_ik gtest)
#add_rostest(test_pr2_arm_ik.xml)

## Check whether there's an X display.  Intended to use in gating tests that
## require a display.
## TODO: after ROS 0.10 is released, remove this re-declaration
#macro(rosbuild_check_for_display var)
#  execute_process(COMMAND "xdpyinfo"
#                  OUTPUT_VARIABLE _dummy
#                  ERROR_VARIABLE _dummy
#                  RESULT_VARIABLE _xdpyinfo_failed)
#  if(_xdpyinfo_failed)
#    set(${var} 0)
#  else(_xdpyinfo_failed)
#    set(${var} 1)
#  endif(_xdpyinfo_failed)
#endmacro(rosbuild_check_for_display)

#if(1)
#  add_executable(pr2_ik_node_regression_test test/test_ik_node_regress.cpp)
#  rosbuild_declare_test(pr2_ik_node_regression_test)
#  target_link_libraries(pr2_ik_node_regression_test pr2_arm_ik gtest)
#  add_rostest(test_pr2_arm_ik_node.xml)
#endif(1)
## Generate added messages and services with any dependencies listed here
generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp angles tf_conversions urdf geometry_msgs kdl_parser moveit_msgs pluginlib eigen moveit_core
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include# TODO include
    LIBRARIES # TODO
)

install(FILES test_pr2_arm_ik.xml test_pr2_arm_ik_node.xml kinematics_plugins.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

