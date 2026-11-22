cmake_minimum_required(VERSION 3.11)
project(small_gicp_relocalization)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
# add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenMP REQUIRED)
ament_auto_find_build_dependencies()

# Add small_gicp dependency
include(FetchContent)
fetchcontent_declare(
  small_gicp
  GIT_REPOSITORY https://github.com/koide3/small_gicp.git
  GIT_TAG "master"
)
fetchcontent_makeavailable(small_gicp)

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX small_gicp)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN small_gicp_relocalization::SmallGicpRelocalizationNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_registration_budget test/test_registration_budget.cpp)
  target_include_directories(test_registration_budget PRIVATE include)
  find_package(PCL REQUIRED COMPONENTS common)
  ament_add_gtest(test_global_sampling test/test_global_sampling.cpp)
  target_compile_features(test_global_sampling PRIVATE cxx_std_17)
  target_include_directories(test_global_sampling PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_global_sampling small_gicp ${PCL_LIBRARIES})
  ament_add_gtest(test_metric_correspondence test/test_metric_correspondence.cpp)
  target_compile_features(test_metric_correspondence PRIVATE cxx_std_17)
  target_include_directories(test_metric_correspondence PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_metric_correspondence small_gicp ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-tidy")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
)
