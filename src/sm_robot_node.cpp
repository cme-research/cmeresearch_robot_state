#include <cmeresearch_robot_state/sm_robot.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  smacc2::run<cmeresearch_robot_state::SmRobot>();
  return 0;
}
