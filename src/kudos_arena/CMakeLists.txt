cmake_minimum_required(VERSION 3.8)
project(kudos_arena)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
add_library(kudos_traffic_light SHARED src/traffic_light_plugin.cpp)
target_compile_features(kudos_traffic_light PUBLIC cxx_std_17)
target_include_directories(kudos_traffic_light PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(kudos_traffic_light ${GAZEBO_LIBRARIES})
ament_target_dependencies(kudos_traffic_light gazebo_ros rclcpp std_msgs)
install(TARGETS kudos_traffic_light LIBRARY DESTINATION lib)

install(
  DIRECTORY config worlds launch models assets
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/make_arena.py scripts/traffic_light_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
