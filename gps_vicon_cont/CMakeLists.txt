cmake_minimum_required(VERSION 2.8.3)
project(gps_vicon_control)
LINK_DIRECTORIES(src/include/vicon)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  mavros
  roscpp
  rospy
  message_generation
)
add_message_files(FILES viconmocap.msg)
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)


include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(viconros src/vicon_src/viconros.cpp src/vicon_src/CFetchViconData.cpp)
target_link_libraries(viconros ${catkin_LIBRARIES} DebugServices ViconDataStreamSDK_CPP)

add_executable(offboard_test src/offb_pos_ctrl/offboard_test.cpp)
target_link_libraries(offboard_test ${catkin_LIBRARIES})

