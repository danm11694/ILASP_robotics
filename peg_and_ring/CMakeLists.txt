cmake_minimum_required(VERSION 2.8.3)
project(peg_and_ring)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++14")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  pcl_conversions
  eigen_conversions
  pcl_ros
  util_pkg
  tf_conversions
  cv_bridge
  image_transport
  pcl_msgs
  dynamic_reconfigure
  message_generation
  dvrk_task_msgs
)


find_package(Boost REQUIRED COMPONENTS system filesystem random date_time thread)
find_package(Eigen3 REQUIRED)

add_message_files(
FILES
PoseStampedArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
#   LIBRARIES 
  CATKIN_DEPENDS roslib roscpp util_pkg pcl_msgs cv_bridge image_transport message_generation dvrk_task_msgs
  #  DEPENDS system_lib
  # DEPENDS boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(peg_ring_node src/peg_ring.cpp src/pcl_conversion_util.cpp)
target_link_libraries( peg_ring_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(converterPCD src/converter_pcd.cpp)
target_link_libraries(converterPCD  ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(combiner src/vrep_combiner.cpp)
target_link_libraries(combiner  ${catkin_LIBRARIES} ${OpenCV_LIBS})

# add_executable(correspondenceRos src/correspondenceRos.cpp)
# target_link_libraries( correspondenceRos ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

# add_executable(registration_pipeline src/registration_pipeline.cpp)
# target_link_libraries( registration_pipeline ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
