cmake_minimum_required(VERSION 3.16)
project(lidar_align)
add_definitions(-w)
set(CMAKE_CXX_STANDARD 14)

# list(APPEND CMAKE_FIND_ROOT_PATH ${PROJECT_SOURCE_DIR})
# set (CMAKE_PREFIX_PATH "/usr/local/lib/cmake/nlopt")
# list(APPEND CMAKE_FIND_ROOT_PATH ${CMAKE_SOURCE_DIR})


# set(image_transport_DIR /opt/ros/noetic/share/image_transport/cmake)
# set(cv_bridge_DIR /opt/ros/noetic/share/cv_bridge/cmake)
# # set(cv_bridge_DIR /opt/ros/noetic/share/cv_bridge/cmake)
# set(pcl_ros_DIR /opt/ros/noetic/share/pcl_ros/cmake)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  pcl_ros
  rosbag
  sensor_msgs
  geometry_msgs
)

# find_package(NLOPT REQUIRED)
find_package(NLopt REQUIRED)

include_directories(
  include
  ${NLOPT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/sensors.cpp
  src/aligner.cpp  
  src/loader.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})

catkin_package(
   INCLUDE_DIRS include/
   LIBRARIES ${PROJECT_NAME}
)

add_executable(lidar_align_node
  src/lidar_align_node.cpp
  
)
target_link_libraries(lidar_align_node ${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/lidar_align
  DESTINATION include
  FILES_MATCHING 
    PATTERN "*.h"
    PATTERN "*.hpp"
)

add_executable(eigentf src/eigentf.cpp)
target_link_libraries(eigentf ${PROJECT_NAME} ${catkin_LIBRARIES})