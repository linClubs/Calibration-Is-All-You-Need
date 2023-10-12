cmake_minimum_required(VERSION 3.0)
project(lidar2cam_calibration)
add_definitions(-w)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  message_filters
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
)

generate_dynamic_reconfigure_options(
  cfg/filter.cfg
)

# set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)
# set(PCL_DIR /usr/lib/x86_64-linux-gnu/cmake/pcl)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

include_directories("/usr/local/include/eigen3/")

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${catkin_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    ${CERES_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    )

# find_package(Sophus REQUIRED)
#$include_directories {Sophus_INCLUDE_DIRS}

catkin_package()

file(GLOB_RECURSE CPP_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_executable(create_aruco_node src/create_aruco_node.cc src/create_aruco.cc)
target_link_libraries(create_aruco_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(data_frame_extraction_node ${CPP_FILE})
target_link_libraries(data_frame_extraction_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(image_procession_node src/image_procession_node.cc src/image_procession.cc)
# target_link_libraries(image_procession_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(estimation_tf_node src/estimation_tf_node.cc src/estimation_tf.cc)
target_link_libraries(estimation_tf_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

