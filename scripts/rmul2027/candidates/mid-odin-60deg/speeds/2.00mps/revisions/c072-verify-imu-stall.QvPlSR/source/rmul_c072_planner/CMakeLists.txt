cmake_minimum_required(VERSION 3.16)
project(rmul_c072_planner)
find_package(ament_cmake REQUIRED)
set(dependencies nav2_core nav2_navfn_planner nav2_costmap_2d nav2_msgs pluginlib rclcpp sensor_msgs
                 nav_msgs tf2 tf2_ros tf2_geometry_msgs dwb_core dwb_msgs
                 builtin_interfaces geometry_msgs std_msgs)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/ObservedObstacle.msg" "msg/ObservedObstacles.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs)
rosidl_get_typesupport_target(c072_tracks_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
add_library(c072_navigation_plugins SHARED
  src/reserve_navfn.cpp
  src/observed_motion_critic.cpp
  src/swept_clearance_critic.cpp
  src/terminal_goal_critic.cpp
  src/path_progress_critic.cpp
  src/inflation_aware_controller.cpp)
target_compile_features(c072_navigation_plugins PUBLIC cxx_std_17)
target_link_libraries(c072_navigation_plugins ${OpenCV_LIBRARIES} "${c072_tracks_typesupport}")
ament_target_dependencies(c072_navigation_plugins ${dependencies})
pluginlib_export_plugin_description_file(nav2_core plugins.xml)
pluginlib_export_plugin_description_file(dwb_core critics.xml)
install(TARGETS c072_navigation_plugins LIBRARY DESTINATION lib)
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
ament_package()
