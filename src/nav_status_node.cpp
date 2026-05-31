#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <action_msgs/msg/goal_status_array.hpp>

using GoalStatusArray = action_msgs::msg::GoalStatusArray;
using GoalStatus = action_msgs::msg::GoalStatus;

class NavStatusNode : public rclcpp::Node
{
public:
  NavStatusNode()
  : Node("nav_status_node")
  {
    pub_ = create_publisher<std_msgs::msg::String>("/nav_status", rclcpp::QoS(1).transient_local());

    sub_ = create_subscription<GoalStatusArray>(
      "/navigate_to_pose/_action/status",
      rclcpp::QoS(10),
      [this](const GoalStatusArray::SharedPtr msg) {
        on_status(msg);
      });

    publish("idle");
    RCLCPP_INFO(get_logger(), "NavStatusNode started, watching /navigate_to_pose/_action/status");
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<GoalStatusArray>::SharedPtr sub_;
  std::string last_status_;

  void on_status(const GoalStatusArray::SharedPtr msg)
  {
    std::string status = "idle";
    for (const auto & goal : msg->status_list) {
      switch (goal.status) {
        case GoalStatus::STATUS_ACCEPTED:
        case GoalStatus::STATUS_EXECUTING:
          status = "navigating";
          break;
        case GoalStatus::STATUS_CANCELING:
          status = "canceling";
          break;
        case GoalStatus::STATUS_CANCELED:
          status = "canceled";
          break;
        case GoalStatus::STATUS_ABORTED:
          status = "aborted";
          break;
        case GoalStatus::STATUS_SUCCEEDED:
          status = "succeeded";
          break;
        default:
          break;
      }
      if (status != "idle") {break;}
    }

    if (status != last_status_) {
      publish(status);
    }
  }

  void publish(const std::string & status)
  {
    last_status_ = status;
    std_msgs::msg::String msg;
    msg.data = status;
    pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Nav status -> %s", status.c_str());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavStatusNode>());
  rclcpp::shutdown();
  return 0;
}
