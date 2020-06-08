cmake_minimum_required(VERSION 2.8.3)
project(px4_gcs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
    roslib
    std_msgs
    message_generation
    message_filters
    serial
)




catkin_package(
)

include_directories(
     include
  ${catkin_INCLUDE_DIRS}
)


add_executable(serial_test src/serial_test.cpp)
add_dependencies(serial_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(serial_test ${catkin_LIBRARIES} )

#add_executable(double_drone_circle src/double_drone_circle.cpp)
#add_dependencies(double_drone_circle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(double_drone_circle ${catkin_LIBRARIES} )

add_executable(drone_unit src/drone_unit.cpp)
add_dependencies(drone_unit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(drone_unit ${catkin_LIBRARIES} )
