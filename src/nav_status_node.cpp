#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <cmeresearch_msgs/msg/nav_status.hpp>

using GoalStatusArray = action_msgs::msg::GoalStatusArray;
using GoalStatus = action_msgs::msg::GoalStatus;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using FeedbackMessage = nav2_msgs::action::NavigateToPose_FeedbackMessage;
using NavStatus = cmeresearch_msgs::msg::NavStatus;

// Publishes the Nav2 navigation status plus live NavigateToPose feedback
// (distance/ETA/recoveries) as a typed cmeresearch_msgs/NavStatus on
// /nav_status. The mqtt_bridge forwards it to the web dashboard.
//
// Status is derived from /navigate_to_pose/_action/status (the action goal
// state machine); the progress fields come from /navigate_to_pose/_action/
// feedback. Fields are zeroed whenever the robot is not actively navigating.
class NavStatusNode : public rclcpp::Node
{
public:
  NavStatusNode()
  : Node("nav_status_node")
  {
    pub_ = create_publisher<NavStatus>("/nav_status", rclcpp::QoS(1).transient_local());

    status_sub_ = create_subscription<GoalStatusArray>(
      "/navigate_to_pose/_action/status", rclcpp::QoS(10),
      [this](const GoalStatusArray::SharedPtr msg) {on_status(msg);});

    feedback_sub_ = create_subscription<FeedbackMessage>(
      "/navigate_to_pose/_action/feedback", rclcpp::QoS(10),
      [this](const FeedbackMessage::SharedPtr msg) {on_feedback(msg);});

    publish();  // seed the latched topic with "idle"
    RCLCPP_INFO(get_logger(),
      "NavStatusNode started, watching /navigate_to_pose/_action/{status,feedback}");
  }

private:
  rclcpp::Publisher<NavStatus>::SharedPtr pub_;
  rclcpp::Subscription<GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::Subscription<FeedbackMessage>::SharedPtr feedback_sub_;

  std::string status_{"idle"};
  float distance_remaining_{0.0f};
  float estimated_time_remaining_{0.0f};
  float navigation_time_{0.0f};
  int16_t number_of_recoveries_{0};

  // Map the goal-status list to one high-level string. Active states
  // (accepted/executing) win over terminal ones regardless of list order; for
  // terminal-only lists the most recent goal (by stamp) decides the outcome.
  static std::string derive_status(const GoalStatusArray::SharedPtr & msg)
  {
    bool active = false;
    bool canceling = false;
    bool have_terminal = false;
    rclcpp::Time newest_terminal(0, 0, RCL_ROS_TIME);
    std::string terminal;

    for (const auto & goal : msg->status_list) {
      switch (goal.status) {
        case GoalStatus::STATUS_ACCEPTED:
        case GoalStatus::STATUS_EXECUTING:
          active = true;
          break;
        case GoalStatus::STATUS_CANCELING:
          canceling = true;
          break;
        case GoalStatus::STATUS_SUCCEEDED:
        case GoalStatus::STATUS_ABORTED:
        case GoalStatus::STATUS_CANCELED: {
            const rclcpp::Time stamp(goal.goal_info.stamp, RCL_ROS_TIME);
            if (!have_terminal || stamp > newest_terminal) {
              have_terminal = true;
              newest_terminal = stamp;
              terminal = (goal.status == GoalStatus::STATUS_SUCCEEDED) ? "succeeded" :
                (goal.status == GoalStatus::STATUS_ABORTED) ? "aborted" :
                "canceled";
            }
            break;
          }
        default:
          break;
      }
    }

    if (active) {return "navigating";}
    if (canceling) {return "canceling";}
    if (have_terminal) {return terminal;}
    return "idle";
  }

  void on_status(const GoalStatusArray::SharedPtr msg)
  {
    const std::string status = derive_status(msg);
    if (status == status_) {return;}
    status_ = status;
    if (status_ != "navigating") {
      // Progress fields are only meaningful while actively navigating.
      distance_remaining_ = 0.0f;
      estimated_time_remaining_ = 0.0f;
      navigation_time_ = 0.0f;
      number_of_recoveries_ = 0;
    }
    publish();
  }

  void on_feedback(const FeedbackMessage::SharedPtr msg)
  {
    const auto & fb = msg->feedback;
    distance_remaining_ = fb.distance_remaining;
    estimated_time_remaining_ =
      static_cast<float>(fb.estimated_time_remaining.sec) +
      static_cast<float>(fb.estimated_time_remaining.nanosec) * 1e-9f;
    navigation_time_ =
      static_cast<float>(fb.navigation_time.sec) +
      static_cast<float>(fb.navigation_time.nanosec) * 1e-9f;
    number_of_recoveries_ = fb.number_of_recoveries;
    // Feedback only flows while a goal is active; keep status consistent even
    // if the status-array message has not arrived yet.
    status_ = "navigating";
    publish();
  }

  void publish()
  {
    NavStatus msg;
    msg.header.stamp = now();
    msg.status = status_;
    msg.distance_remaining = distance_remaining_;
    msg.estimated_time_remaining = estimated_time_remaining_;
    msg.navigation_time = navigation_time_;
    msg.number_of_recoveries = number_of_recoveries_;
    pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Nav status -> %s (dist=%.2f m, eta=%.1f s, recoveries=%d)",
      status_.c_str(), distance_remaining_, estimated_time_remaining_, number_of_recoveries_);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavStatusNode>());
  rclcpp::shutdown();
  return 0;
}
