#include "vica_nav2_bt_plugins/is_rail_ahead_clear_condition.hpp"

#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace vica_nav2_bt_plugins
{

IsRailAheadClearCondition::IsRailAheadClearCondition(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  // bt_navigator 의 client 노드는 실행기에 안 붙어 있다. nav2 의 IsBatteryLow 처럼 따로 묶고
  // tick 마다 spin_some 으로 받은 것만 꺼낸다.
  group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(group_, node_->get_node_base_interface());

  std::string topic = "global_costmap/costmap_raw";
  getInput("costmap_topic", topic);
  int need = 2;
  getInput("confirm_ticks", need);
  confirm_ = BlockedConfirm(need);

  rclcpp::SubscriptionOptions opt;
  opt.callback_group = group_;
  // costmap 발행 QoS 와 맞춘다(transient_local·reliable, depth 1).
  sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    topic, rclcpp::QoS(1).transient_local().reliable(),
    [this](nav2_msgs::msg::Costmap::SharedPtr msg) {
      grid_.size_x = msg->metadata.size_x;
      grid_.size_y = msg->metadata.size_y;
      grid_.resolution = msg->metadata.resolution;
      grid_.origin_x = msg->metadata.origin.position.x;
      grid_.origin_y = msg->metadata.origin.position.y;
      grid_.data = msg->data;
      // 받은 시각으로 신선도를 본다. 헤더 시각에 기대면 발행 쪽 시계·빈 stamp 하나로 늘 '오래됨'(=늘 막힘)이 된다.
      // 발행이 끊기면 콜백이 안 불려 이 값이 멈추므로 끊김은 그대로 잡힌다.
      grid_stamp_ = node_->now();
      have_grid_ = true;
    }, opt);
}

BT::NodeStatus IsRailAheadClearCondition::tick()
{
  executor_.spin_some();

  std::string text;
  double pad = 0.05;
  getInput("body_footprint", text);
  getInput("footprint_padding", pad);
  if (text != body_text_) {
    body_text_ = text;
    body_ = padPolygon(parsePolygon(text), pad);
  }
  if (body_.size() < 3) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "[IsRailAheadClear] body_footprint 를 못 읽었다('%s'). 레일을 막힘으로 본다", text.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const rclcpp::Time now = node_->now();
  double max_age = 3.0;
  getInput("max_costmap_age", max_age);
  if (!have_grid_ || (now - grid_stamp_).seconds() > max_age) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "[IsRailAheadClear] global costmap 이 없거나 %.1f s 넘게 오래됐다 -> 레일을 막힘으로 본다",
      max_age);
    return BT::NodeStatus::FAILURE;
  }

  nav_msgs::msg::Path path;
  if (!getInput("path", path) || path.poses.size() < 2) {
    return BT::NodeStatus::FAILURE;
  }
  double skip = 0.5;
  getInput("skip_distance", skip);

  const auto v = checkRailAhead(path, skip, body_, grid_);
  const bool confirmed = confirm_.update(v.blocked, now.seconds());
  if (v.blocked) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[IsRailAheadClear] 레일 앞 %.2f m 에서 몸통이 장애물(%.2f, %.2f)에 닿는다 — %d번째%s",
      v.hit_s, v.hit_x, v.hit_y, confirm_.count(), confirmed ? " -> 막힘" : "(아직 레일 유지)");
  }
  return confirmed ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

}  // namespace vica_nav2_bt_plugins

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<vica_nav2_bt_plugins::IsRailAheadClearCondition>("IsRailAheadClear");
}
