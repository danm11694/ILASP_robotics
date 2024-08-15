cmake_minimum_required(VERSION 2.8.3)
project(util_pkg)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS pcl_ros pcl_conversions std_srvs ann_catkin cmake_modules image_transport cv_bridge)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem random date_time thread)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ood_vis misc_utils
  CATKIN_DEPENDS pcl_ros pcl_conversions ann_catkin cmake_modules cv_bridge
  DEPENDS eigen boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})



## Declare a cpp library
add_library(misc_utils src/io_utils.cpp src/pcd_utils.cpp src/misc.cpp)
add_library(ood_vis src/ood_visualization.cpp)
add_library(tabletop_analyzer src/tabletop_color_analyzer.cpp)



add_executable(test_boost_dd_sampler src/test_boost_dd_sampler.cpp)


add_executable(combiner src/vrep_combiner.cpp)
target_link_libraries(combiner  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(combine_depth_rgb src/combine_depth_rbg.cpp)
target_link_libraries(combine_depth_rgb  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(evaluate_calibration src/evaluate_calibration.cpp)
target_link_libraries(evaluate_calibration  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(converter_pcl src/converter_pcl.cpp)
target_link_libraries(converter_pcl ${Boost_LIBRARIES} ${catkin_LIBRARIES})


# add_executable(createMesh src/createMesh.cpp)
# target_link_libraries(createMesh ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(color_segmentation src/color_segmentation.cpp src/pcd_utils.cpp)
target_link_libraries(color_segmentation  ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(needle_semantic_segmentation src/needle_semantic_segmentation.cpp)
target_link_libraries(needle_semantic_segmentation  ${catkin_LIBRARIES} ${OpenCV_LIBS})



add_executable(chessboard_rec src/chessboard_rec.cpp)
target_link_libraries(chessboard_rec  ${catkin_LIBRARIES} ${OpenCV_LIBS})

target_link_libraries(test_boost_dd_sampler ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(misc_utils ${catkin_LIBRARIES})
target_link_libraries(ood_vis ${catkin_LIBRARIES})