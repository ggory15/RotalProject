cmake_minimum_required(VERSION 3.5)
project(nav2_collision_monitor)

### Dependencies ###

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

### Header ###

nav2_package()

### Libraries and executables ###

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_util
  nav2_costmap_2d
)

set(executable_name collision_monitor)
set(executable_name2 elevator_monitor)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/collision_monitor_node.cpp
  src/polygon.cpp
  src/circle.cpp
  src/source.cpp
  src/scan.cpp
  src/pointcloud.cpp
  src/kinematics.cpp
)

add_executable(${executable_name}
  src/main.cpp
)

add_executable(${executable_name2}
  src/main2.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${executable_name}
  ${library_name}
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name2}
  ${library_name}
)

ament_target_dependencies(${executable_name2}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "nav2_collision_monitor::CollisionMonitor")

### Install ###

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${executable_name2}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})


### Ament stuff ###

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
