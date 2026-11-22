cmake_minimum_required(VERSION 3.16)
project(sensor_scan_generation)

find_package(ament_cmake REQUIRED)
set(dependencies rclcpp rclcpp_components message_filters nav_msgs sensor_msgs tf2_sensor_msgs
                 tf2 tf2_ros tf2_geometry_msgs)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(sensor_scan_generation SHARED src/sensor_scan_generation.cpp)
target_compile_features(sensor_scan_generation PUBLIC cxx_std_17)
target_include_directories(sensor_scan_generation PRIVATE include)
ament_target_dependencies(sensor_scan_generation ${dependencies})
rclcpp_components_register_nodes(sensor_scan_generation
  "sensor_scan_generation::SensorScanGenerationNode")

install(TARGETS sensor_scan_generation LIBRARY DESTINATION lib)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
