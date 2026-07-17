cmake_minimum_required(VERSION 3.8)
project(cmeresearch_robot_state)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(smacc2 REQUIRED)
find_package(smacc2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(cmeresearch_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include)

add_executable(sm_robot_node src/sm_robot_node.cpp)
target_link_libraries(sm_robot_node
  ${Boost_LIBRARIES}
)

ament_target_dependencies(sm_robot_node
  rclcpp
  smacc2
  smacc2_msgs
  std_msgs
  cmeresearch_msgs
)

add_executable(nav_status_node src/nav_status_node.cpp)
ament_target_dependencies(nav_status_node
  rclcpp
  action_msgs
  nav2_msgs
  cmeresearch_msgs
)

add_executable(system_stats_node src/system_stats_node.cpp)
ament_target_dependencies(system_stats_node
  rclcpp
  cmeresearch_msgs
)

add_executable(map_pose_node src/map_pose_node.cpp)
ament_target_dependencies(map_pose_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

install(TARGETS
  sm_robot_node
  nav_status_node
  system_stats_node
  map_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_state_transitions.py)
endif()

ament_package()
