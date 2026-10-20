#include <csignal>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "vica_interfaces/msg/turn_guide.hpp"

#include "vica_user_guidance_cpp/pyfmt.hpp"
#include "vica_user_guidance_cpp/turn_core.hpp"

namespace vica_guidance
{

namespace
{
builtin_interfaces::msg::Time to_msg(int64_t ns)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(ns / 1000000000);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000);
  return t;
}
}

class TurnGuideNode : public rclcpp::Node, public TurnIo
{
public:
  TurnGuideNode()
  : rclcpp::Node("turn_guide_node"),
    steady_clock_(RCL_STEADY_TIME)
  {
    TurnParams p;
    p.odom_topic = declare_parameter<std::string>("odom_topic", p.odom_topic);
    p.window_sec = declare_parameter<double>("window_sec", p.window_sec);
    p.enter_threshold_deg = declare_parameter<double>("enter_threshold_deg", p.enter_threshold_deg);
    p.exit_threshold_deg = declare_parameter<double>("exit_threshold_deg", p.exit_threshold_deg);
    p.min_duration_sec = declare_parameter<double>("min_duration_sec", p.min_duration_sec);
    p.odom_timeout_sec = declare_parameter<double>("odom_timeout_sec", p.odom_timeout_sec);
    p.publish_rate_hz = declare_parameter<double>("publish_rate_hz", p.publish_rate_hz);
    p.cue_valid_sec = declare_parameter<double>("cue_valid_sec", p.cue_valid_sec);

    p.enable_rail_forecast =
      declare_parameter<bool>("enable_rail_forecast", p.enable_rail_forecast);
    p.rail_topic = declare_parameter<std::string>("rail_topic", p.rail_topic);
    p.rail_timeout_sec = declare_parameter<double>("rail_timeout_sec", p.rail_timeout_sec);
    p.map_frame = declare_parameter<std::string>("map_frame", p.map_frame);
    p.base_frame = declare_parameter<std::string>("base_frame", p.base_frame);
    p.rail_corner_threshold_deg =
      declare_parameter<double>("rail_corner_threshold_deg", p.rail_corner_threshold_deg);
    p.rail_lead_time_sec = declare_parameter<double>("rail_lead_time_sec", p.rail_lead_time_sec);
    p.rail_max_offtrack_m = declare_parameter<double>("rail_max_offtrack_m", p.rail_max_offtrack_m);
    p.rail_handoff_m = declare_parameter<double>("rail_handoff_m", p.rail_handoff_m);
    p.rail_merge_gap_m = declare_parameter<double>("rail_merge_gap_m", p.rail_merge_gap_m);
    p.rail_goal_change_m = declare_parameter<double>("rail_goal_change_m", p.rail_goal_change_m);
    p.rail_behind_start_m = declare_parameter<double>("rail_behind_start_m", p.rail_behind_start_m);
    p.rail_min_prepare_dist_m =
      declare_parameter<double>("rail_min_prepare_dist_m", p.rail_min_prepare_dist_m);
    p.rail_rehold_end_m = declare_parameter<double>("rail_rehold_end_m", p.rail_rehold_end_m);
    p.controller_lookahead_time_sec = declare_parameter<double>(
      "controller_lookahead_time_sec", p.controller_lookahead_time_sec);
    p.controller_lookahead_min_m =
      declare_parameter<double>("controller_lookahead_min_m", p.controller_lookahead_min_m);
    p.controller_lookahead_max_m =
      declare_parameter<double>("controller_lookahead_max_m", p.controller_lookahead_max_m);
    p.minor_turn_deg = declare_parameter<double>("minor_turn_deg", p.minor_turn_deg);
    p.minor_turn_min_speed_mps =
      declare_parameter<double>("minor_turn_min_speed_mps", p.minor_turn_min_speed_mps);

    core_ = std::make_unique<TurnCore>(p, *this);

    if (core_->rail_enabled()) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
      sub_rail_ = create_subscription<nav_msgs::msg::Path>(
        p.rail_topic, 1, [this](const nav_msgs::msg::Path & m) {
          std::vector<Point> xy;
          xy.reserve(m.poses.size());
          for (const auto & ps : m.poses) {xy.emplace_back(ps.pose.position.x, ps.pose.position.y);}
          core_->on_rail(m.header.frame_id, xy);
        });
      core_->log_rail_forecast_on();
    }

    pub_guide_ = create_publisher<vica_interfaces::msg::TurnGuide>("/vica/turn_guide", 10);
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      p.odom_topic, 10, [this](const nav_msgs::msg::Odometry & m) {
        const auto & q = m.pose.pose.orientation;
        core_->on_odom(q.x, q.y, q.z, q.w, m.twist.twist.linear.x);
      });
    timer_ = create_wall_timer(
      std::chrono::nanoseconds(core_->publish_period_ns()), [this] {core_->publish_loop();});
    core_->log_started();
  }

  void publish_idle_once() {core_->publish_idle_once();}

  int64_t steady_now_ns() override {return steady_clock_.now().nanoseconds();}
  int64_t ros_now_ns() override {return get_clock()->now().nanoseconds();}

  void log(LogLevel level, const std::string & text) override
  {
    switch (level) {
      case LogLevel::Info: RCLCPP_INFO(get_logger(), "%s", text.c_str()); break;
      case LogLevel::Warn: RCLCPP_WARN(get_logger(), "%s", text.c_str()); break;
      case LogLevel::Error: RCLCPP_ERROR(get_logger(), "%s", text.c_str()); break;
    }
  }

  void publish_guide(const GuideOut & g) override
  {
    vica_interfaces::msg::TurnGuide msg;
    msg.header.stamp = to_msg(g.stamp_ns);
    msg.header.frame_id = g.frame_id;
    msg.direction = g.direction;
    msg.phase = g.phase;
    msg.distance_m = g.distance_m;
    msg.turn_angle_deg = g.turn_angle_deg;
    msg.sequence_id = g.sequence_id;
    msg.valid_until = to_msg(g.valid_until_ns);
    msg.source_stale = g.source_stale;
    pub_guide_->publish(msg);
  }

  std::optional<TfPose> lookup_transform(
    const std::string & target, const std::string & source, std::string & error) override
  {
    try {
      const auto t = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
      TfPose out;
      out.x = t.transform.translation.x;
      out.y = t.transform.translation.y;
      out.qx = t.transform.rotation.x;
      out.qy = t.transform.rotation.y;
      out.qz = t.transform.rotation.z;
      out.qw = t.transform.rotation.w;
      return out;
    } catch (const tf2::TransformException & e) {
      error = e.what();
      return std::nullopt;
    }
  }

private:
  rclcpp::Clock steady_clock_;
  std::unique_ptr<TurnCore> core_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_rail_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<vica_interfaces::msg::TurnGuide>::SharedPtr pub_guide_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}

int main(int argc, char ** argv)
{
  std::signal(SIGPIPE, SIG_IGN);
  rclcpp::init(argc, argv);
  std::shared_ptr<vica_guidance::TurnGuideNode> node;
  try {
    node = std::make_shared<vica_guidance::TurnGuideNode>();
  } catch (const vica_guidance::PyError & e) {
    std::fprintf(stderr, "%s: %s\n", e.type().c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  } catch (const std::exception & e) {
    std::fprintf(stderr, "%s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  int rc = 0;
  try {
    executor.spin();
  } catch (const vica_guidance::PyError & e) {
    std::fprintf(stderr, "%s: %s\n", e.type().c_str(), e.what());
    rc = 1;
  } catch (const std::exception & e) {
    std::fprintf(stderr, "%s\n", e.what());
    rc = 1;
  }
  executor.remove_node(node);
  try {
    node->publish_idle_once();
  } catch (...) {
  }
  node.reset();
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return rc;
}
