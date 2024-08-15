cmake_minimum_required (VERSION 2.8.3)

project (robot_control)

## find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package (catkin REQUIRED
              COMPONENTS
              geometry_msgs
              roscpp
              std_msgs
              roslib
              dvrk_robot 
              dvrk_task_msgs
              )

find_package (cisst 1.0.11 REQUIRED ${REQUIRED_CISST_LIBRARIES})

catkin_package (CATKIN_DEPENDS geometry_msgs dvrk_task_msgs roscpp std_msgs smach smach_ros dvrk_robot)
