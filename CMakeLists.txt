cmake_minimum_required(VERSION 3.8)
project(birdro_planner)

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(birdro_planner SHARED
  src/birdro_planner.cpp
)

rclcpp_components_register_node(birdro_planner
  PLUGIN "birdro_planner::BirdroPlanner"
  EXECUTABLE birdro_planner_node
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

target_link_libraries(${PROJECT_NAME}
  tinyxml2::tinyxml2
)

add_executable(ndz_costmap_overlay
  src/ndz_costmap_overlay.cpp
)
ament_target_dependencies(ndz_costmap_overlay rclcpp nav_msgs visualization_msgs geometry_msgs tier4_map_msgs geography_utils)
target_link_libraries(ndz_costmap_overlay tinyxml2::tinyxml2)
install(TARGETS ndz_costmap_overlay DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
