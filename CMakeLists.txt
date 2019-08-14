cmake_minimum_required(VERSION 2.8.3)
project(tfpose_ros)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

# setup.py is called during catkin build

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Keypoint.msg
  Pose.msg
  Poses.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy message_generation message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set( OpenCV_DIR "/usr/local/include/opencv2/" )
find_package( OpenCV REQUIRED )

set(CMAKE_CXX_FLAGS "-pthread")

add_executable(tf_pose_kinect src/kinect.cpp)
add_dependencies(tf_pose_kinect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_pose_kinect ${OpenCV_LIBS} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(tf_pose_realsense src/realsense.cpp)
add_dependencies(tf_pose_realsense ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_pose_realsense ${OpenCV_LIBS} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
