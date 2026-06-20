#ifndef SM_ROBOT_HPP
#define SM_ROBOT_HPP

#include <chrono>
#include <map>
#include <sstream>
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

static const std::vector<std::string> DEFAULT_DRIVER_STATE_TOPICS = {
  "/cmexa_base/front_left/state",
  "/cmexa_base/front_right/state",
  "/cmexa_base/rear_left/state",
  "/cmexa_base/rear_right/state",
};

// Returns the second-to-last `/`-separated segment of `topic`.
// For "/cmexa_base/front_left/state" → "front_left". Used to populate
// RobotState.driver_names with the wheel identifier instead of the trailing
// "/state" leaf, which is what consumers (e.g. the webapp's robot-state
// panel) actually need to label each driver row.
inline std::string wheel_name_from_topic(const std::string & topic)
{
  std::vector<std::string> segments;
  std::stringstream ss(topic);
  std::string seg;
  while (std::getline(ss, seg, '/')) {
    if (!seg.empty()) {
      segments.push_back(seg);
    }
  }
  if (segments.size() >= 2) {
    return segments[segments.size() - 2];
  }
  return topic;
}

// STATE MACHINE
struct SmRobot : public smacc2::SmaccStateMachineBase<SmRobot, StateInitializing>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  rclcpp::Publisher<cmeresearch_msgs::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> driver_subs_;
  std::map<std::string, std::string> driver_states_;
  std::vector<std::string> driver_topics_;
  rclcpp::TimerBase::SharedPtr init_timeout_timer_;
  bool initialization_complete_ = false;
  double init_timeout_sec_ = 30.0;

  void onInitialize() override
  {
    RCLCPP_INFO(getLogger(), "SmRobot: Initializing...");

    auto node = getNode();
    // `driver_topics` lets sim / dev workspaces override the 4 hardware stepper
    // driver state topics — pass `[]` to skip readiness gating entirely so the
    // state machine reaches Idle and publishes RobotState even when no
    // tinkerforge drivers are around (e.g. in Webots sim).
    driver_topics_ = node->declare_parameter<std::vector<std::string>>(
      "driver_topics", DEFAULT_DRIVER_STATE_TOPICS);
    // `init_timeout_sec` is the hard cap on how long we wait for all
    // configured drivers to report `initialized`/`idle` before forcing a
    // transition to Idle anyway. Set to 0 to disable the timeout.
    init_timeout_sec_ = node->declare_parameter<double>("init_timeout_sec", 30.0);

    state_pub_ = node->create_publisher<cmeresearch_msgs::msg::RobotState>(
      "/robot_state", rclcpp::QoS(1).transient_local());

    cmd_sub_ = node->create_subscription<std_msgs::msg::String>(
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
    for (const auto & topic : driver_topics_) {
      driver_states_[topic] = "";
      auto sub = node->create_subscription<std_msgs::msg::String>(
        topic, driver_qos,
        [this, topic](const std_msgs::msg::String::SharedPtr msg) {
          const std::string prev = driver_states_[topic];
          driver_states_[topic] = msg->data;
          RCLCPP_INFO(getLogger(), "Driver [%s]: %s", topic.c_str(), msg->data.c_str());
          if (msg->data == "low_voltage" && prev != "low_voltage") {
            RCLCPP_ERROR(getLogger(), "Low voltage on [%s] — triggering emergency stop",
              topic.c_str());
            publishState("emergency_stop");
            this->postEvent<EvEmergencyStop>();
          } else {
            checkDriversAndPost();
          }
        });
      driver_subs_.push_back(sub);
    }

    // First-time entry into the Initializing state on bringup.
    beginInitialization();
  }

  // Arm (or re-arm) the init timeout backstop. Idempotent — cancels any
  // previously running timer first so a reset->Initializing re-entry gets
  // a fresh `init_timeout_sec_` window rather than firing immediately on
  // the stale handle.
  void armInitTimeout()
  {
    init_timeout_timer_.reset();
    if (init_timeout_sec_ <= 0.0) {
      return;
    }
    const auto period =
      std::chrono::milliseconds(static_cast<int64_t>(init_timeout_sec_ * 1000.0));
    init_timeout_timer_ = getNode()->create_wall_timer(
      period,
      [this]() {
        if (initialization_complete_) {
          init_timeout_timer_.reset();
          return;
        }
        RCLCPP_WARN(getLogger(),
          "Init timeout (%.1fs) reached before all drivers reported ready — "
          "forcing transition to idle. Driver states:", init_timeout_sec_);
        for (const auto & kv : driver_states_) {
          RCLCPP_WARN(getLogger(), "  %s = '%s'",
            kv.first.c_str(),
            kv.second.empty() ? "<no message>" : kv.second.c_str());
        }
        initialization_complete_ = true;
        this->postEvent<EvStateFinished>();
        init_timeout_timer_.reset();
      });
  }

  // Reset the readiness latch, arm the timer, and immediately try to
  // transition out. Called both from `onInitialize` (first entry) and
  // from `StateInitializing::runtimeConfigure` (re-entry after reset).
  void beginInitialization()
  {
    initialization_complete_ = false;
    armInitTimeout();
    checkDriversAndPost();
  }

  void checkDriversAndPost()
  {
    if (initialization_complete_) {
      return;
    }
    if (driver_topics_.empty()) {
      RCLCPP_INFO(getLogger(), "No driver topics configured, transitioning to idle");
      initialization_complete_ = true;
      this->postEvent<EvStateFinished>();
      return;
    }
    for (const auto & kv : driver_states_) {
      if (kv.second == "error") {
        RCLCPP_ERROR(getLogger(), "Driver error on [%s], staying in initializing",
            kv.first.c_str());
        publishState("driver_error");
        return;
      }
      if (kv.second != "initialized" && kv.second != "idle") {
        return;
      }
    }
    RCLCPP_INFO(getLogger(), "All drivers ready, transitioning to idle");
    initialization_complete_ = true;
    this->postEvent<EvStateFinished>();
  }

  void publishState(const std::string & state_name)
  {
    if (!state_pub_) {return;}
    cmeresearch_msgs::msg::RobotState msg;
    msg.header.stamp = getNode()->get_clock()->now();
    msg.header.frame_id = "robot";
    msg.state = state_name;
    for (const auto & topic : driver_topics_) {
      msg.driver_names.push_back(wheel_name_from_topic(topic));
      const auto it = driver_states_.find(topic);
      msg.driver_states.push_back(it != driver_states_.end() ? it->second : "unknown");
    }
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
    // Re-arm the timeout backstop and reset the readiness latch on every
    // entry. Without this the Initializing state is one-shot — on a
    // reset->Initializing re-entry, `initialization_complete_` stays
    // `true`, `checkDriversAndPost` returns immediately, and no
    // EvStateFinished ever fires (test_04_reset_from_emergency_stop).
    dynamic_cast<SmRobot &>(this->getStateMachine()).beginInitialization();
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
