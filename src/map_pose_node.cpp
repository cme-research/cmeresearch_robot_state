#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/time.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

// Looks up the map->base_link transform and republishes it as a PoseStamped on
// /robot_pose at a fixed rate. Consumers without a TF tree (the web dashboard,
// via mqtt_bridge) can then show the robot's true pose on the map. Works in
// both mapping (slam_toolbox provides map->odom) and localization (AMCL) modes;
// while the transform is unavailable (SLAM/AMCL still starting) it simply
// publishes nothing.
class MapPoseNode : public rclcpp::Node
{
public:
  MapPoseNode()
  : Node("map_pose_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const double rate = declare_parameter<double>("publish_rate", 10.0);

    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/robot_pose", rclcpp::QoS(1));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      [this]() {on_timer();});

    RCLCPP_INFO(get_logger(), "MapPoseNode publishing %s->%s as /robot_pose at %.1f Hz",
      map_frame_.c_str(), base_frame_.c_str(), rate);
  }

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string map_frame_;
  std::string base_frame_;

  void on_timer()
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(get_logger(), "TF %s->%s unavailable: %s",
        map_frame_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = tf.header;  // frame_id = map_frame_, latest stamp
    pose.pose.position.x = tf.transform.translation.x;
    pose.pose.position.y = tf.transform.translation.y;
    pose.pose.position.z = tf.transform.translation.z;
    pose.pose.orientation = tf.transform.rotation;
    pub_->publish(pose);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPoseNode>());
  rclcpp::shutdown();
  return 0;
}
