# Copyright 2026 Theo Morel
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(mine_explorer_mapping)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)

install(DIRECTORY
  launch
  config
  src
  DESTINATION share/${PROJECT_NAME}
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(elevation_mapping_node
  src/elevation_mapping_node.cpp
)

ament_target_dependencies(
  elevation_mapping_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  grid_map_ros
  grid_map_msgs
)

target_link_libraries(
  elevation_mapping_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  elevation_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(mine_explorer_gazebo REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_slam_2d_integration test/test_slam_2d_integration.py)
endif()

ament_package()