cmake_minimum_required(VERSION 3.16)
project(point_time_contract)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenMP REQUIRED)
get_filename_component(NAV_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
add_executable(point_time_contract contract.cpp
  "${NAV_ROOT}/point_lio/src/preprocess.cpp"
  "${NAV_ROOT}/ign_sim_pointcloud_tool/src/point_cloud_converter.cpp")
target_include_directories(point_time_contract PRIVATE
  "${NAV_ROOT}/point_lio/src"
  "${NAV_ROOT}/ign_sim_pointcloud_tool/include" ${PCL_INCLUDE_DIRS})
ament_target_dependencies(point_time_contract rclcpp rclcpp_components pcl_conversions livox_ros_driver2)
target_link_libraries(point_time_contract ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)
