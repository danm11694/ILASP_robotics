cmake_minimum_required (VERSION 2.8.3)

project (dvrk_task_msgs)

## find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package (catkin REQUIRED
              COMPONENTS
              geometry_msgs
              roscpp
              std_msgs
              roslib
              message_generation
              )

add_message_files(FILES
    CloudArray.msg
    ObstArray.msg
    IntArray.msg
    PoseStampedArray.msg
    BoolArray.msg
    ContextInput.msg
    ContextModel.msg
    ActionRequestFiner.msg 
    ActionArray.msg 
    ActionFeedback.msg
    ILPExample.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

find_package (cisst 1.0.11 REQUIRED ${REQUIRED_CISST_LIBRARIES})

catkin_package (CATKIN_DEPENDS geometry_msgs roscpp std_msgs)
