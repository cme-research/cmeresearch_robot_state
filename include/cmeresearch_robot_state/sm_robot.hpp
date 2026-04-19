#ifndef SM_ROBOT_HPP
#define SM_ROBOT_HPP

#include <map>
#include <string>
#include <vector>

#include <smacc2/smacc.hpp>
#include <boost/mpl/list.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmeresearch_msgs/msg/robot_state.hpp>

namespace cmeresearch_robot_state
{

// EVENTS
struct EvStateFinished : sc::event<EvStateFinished> {};
struct EvMissionStart : sc::event<EvMissionStart> {};
struct EvEmergencyStop : sc::event<EvEmergencyStop> {};
struct EvReset : sc::event<EvReset> {};

// STATES
struct StateInitializing;
struct StateIdle;
struct StateMoving;
struct StateEmergencyStop;

static const std::vector<std::string> DRIVER_STATE_TOPICS = {
  "/cmexa_base/front_left/state",
  "/cmexa_base/front_right/state",
  "/cmexa_base/rear_left/state",
  "/cmexa_base/rear_right/state",
};

// STATE MACHINE
struct SmRobot : public smacc2::SmaccStateMachineBase<SmRobot, StateInitializing>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  rclcpp::Publisher<cmeresearch_msgs::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> driver_subs_;
  std::map<std::string, std::string> driver_states_;

  void onInitialize() override
  {
    RCLCPP_INFO(getLogger(), "SmRobot: Initializing...");

    state_pub_ = getNode()->create_publisher<cmeresearch_msgs::msg::RobotState>(
      "/robot_state", rclcpp::QoS(1).transient_local());

    cmd_sub_ = getNode()->create_subscription<std_msgs::msg::String>(
      "/robot_cmd", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(getLogger(), "Received command: %s", msg->data.c_str());
        if (msg->data == "start_mission") {
          this->postEvent<EvMissionStart>();
        } else if (msg->data == "emergency_stop") {
          this->postEvent<EvEmergencyStop>();
        } else if (msg->data == "reset") {
          this->postEvent<EvReset>();
        }
      });

    const rclcpp::QoS driver_qos = rclcpp::QoS(1).transient_local();
    for (const auto & topic : DRIVER_STATE_TOPICS) {
      driver_states_[topic] = "";
      auto sub = getNode()->create_subscription<std_msgs::msg::String>(
        topic, driver_qos,
        [this, topic](const std_msgs::msg::String::SharedPtr msg) {
          driver_states_[topic] = msg->data;
          RCLCPP_INFO(getLogger(), "Driver [%s]: %s", topic.c_str(), msg->data.c_str());
          checkDriversAndPost();
        });
      driver_subs_.push_back(sub);
    }
  }

  void checkDriversAndPost()
  {
    for (const auto & kv : driver_states_) {
      if (kv.second == "error") {
        RCLCPP_ERROR(getLogger(), "Driver error on [%s], staying in initializing", kv.first.c_str());
        publishState("driver_error");
        return;
      }
      if (kv.second != "initialized") {
        return;
      }
    }
    RCLCPP_INFO(getLogger(), "All drivers initialized, transitioning to idle");
    this->postEvent<EvStateFinished>();
  }

  void publishState(const std::string & state_name)
  {
    if (!state_pub_) {return;}
    cmeresearch_msgs::msg::RobotState msg;
    msg.header.stamp = getNode()->get_clock()->now();
    msg.header.frame_id = "robot";
    msg.state = state_name;
    state_pub_->publish(msg);
    RCLCPP_INFO(getLogger(), "State -> %s", state_name.c_str());
  }
};

// INITIALIZING STATE
struct StateInitializing : public smacc2::SmaccState<StateInitializing, SmRobot>
{
  using SmaccState::SmaccState;

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "Entering Initializing State — waiting for all 4 drivers");
    dynamic_cast<SmRobot &>(this->getStateMachine()).publishState("initializing");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "Exiting Initializing State");
  }

  typedef boost::mpl::list<
      smacc2::Transition<EvStateFinished, StateIdle>
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure()
  {
    dynamic_cast<SmRobot &>(this->getStateMachine()).checkDriversAndPost();
  }
};

// IDLE STATE
struct StateIdle : public smacc2::SmaccState<StateIdle, SmRobot>
{
  using SmaccState::SmaccState;

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "Entering Idle State");
    dynamic_cast<SmRobot &>(this->getStateMachine()).publishState("idle");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "Exiting Idle State");
  }

  typedef boost::mpl::list<
      smacc2::Transition<EvMissionStart, StateMoving>,
      smacc2::Transition<EvEmergencyStop, StateEmergencyStop>
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}
};

// MOVING STATE
struct StateMoving : public smacc2::SmaccState<StateMoving, SmRobot>
{
  using SmaccState::SmaccState;

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "Entering Moving State");
    dynamic_cast<SmRobot &>(this->getStateMachine()).publishState("moving");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "Exiting Moving State");
  }

  typedef boost::mpl::list<
      smacc2::Transition<EvStateFinished, StateIdle>,
      smacc2::Transition<EvEmergencyStop, StateEmergencyStop>
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}
};

// EMERGENCY STOP STATE
struct StateEmergencyStop : public smacc2::SmaccState<StateEmergencyStop, SmRobot>
{
  using SmaccState::SmaccState;

  void onEntry()
  {
    RCLCPP_ERROR(getLogger(), "EMERGENCY STOP");
    dynamic_cast<SmRobot &>(this->getStateMachine()).publishState("emergency_stop");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "Exiting Emergency Stop State");
  }

  typedef boost::mpl::list<
      smacc2::Transition<EvReset, StateInitializing>
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}
};

}  // namespace cmeresearch_robot_state

#endif  // SM_ROBOT_HPP
