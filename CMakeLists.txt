cmake_minimum_required(VERSION 3.8)
project(cmeresearch_robot_state)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(smacc2 REQUIRED)
find_package(smacc2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cmeresearch_msgs REQUIRED)
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

install(TARGETS
  sm_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
