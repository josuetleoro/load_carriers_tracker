cmake_minimum_required(VERSION 3.0.2)
project(load_carriers_tracker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf2
  unique_id
)

# find_package(Boost REQUIRED COMPONENTS system)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(transform_detections src/transform_detections.cpp)
add_dependencies(transform_detections ${catkin_EXPORTED_TARGETS})
target_link_libraries(transform_detections ${catkin_LIBRARIES})

add_executable(carrier_tracker_node src/carriers_tracker_node.cpp src/carriers_tracker.cpp src/object_tracker.cpp)
add_dependencies(carrier_tracker_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(carrier_tracker_node ${catkin_LIBRARIES})