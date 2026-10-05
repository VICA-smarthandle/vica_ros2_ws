// BT 조건 노드 <IsRailAheadClear path="{rail_ahead}" .../>
//
// 레일 트리에서 Nav2 IsPathValid 를 대신한다(2026-10-05 해결안 B, rail_clear_check.hpp 근거).
// 레일 앞 조각의 첫 skip_distance 는 건너뛰고, 손잡이 꼬리를 뺀 몸통(body_footprint)이 global costmap 의
// 치명 칸과 겹치는지 본다. 막힘이 confirm_ticks 번 이어져야 FAILURE(=레일 포기)다.
// 로봇 바로 옆 장애물·코너에서 꼬리가 휘는 것은 VCC 가 몸 충돌 검사·유턴 여유 검사로 초당 10번 본다 —
// 이 노드는 "앞길에 누가 서 있나"만 맡는다.
//
// costmap 은 global_costmap/costmap_raw(nav2_msgs/Costmap, publish_frequency 1 Hz)를 구독해 받는다.
// IsPathValid 는 planner 의 살아 있는 costmap 을 봤으므로 최대 1 s 늦다. 받은 적이 없거나
// max_costmap_age 보다 오래되면 FAILURE(막힘 쪽) — planner 가 살아 있는 costmap 으로 당근 경로를 그린다.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/condition_node.h"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vica_nav2_bt_plugins/rail_clear_check.hpp"

namespace vica_nav2_bt_plugins
{

class IsRailAheadClearCondition : public BT::ConditionNode
{
public:
  IsRailAheadClearCondition(const std::string & name, const BT::NodeConfiguration & conf);
  IsRailAheadClearCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "검사할 레일 앞 조각 (TruncatePathLocal 출력)"),
      BT::InputPort<double>("skip_distance", 0.5,
        "조각 첫 점에서 이만큼(m)은 건너뛴다. 로봇이 지금 서 있는 자리"),
      BT::InputPort<std::string>("body_footprint", "",
        "몸통 다각형 \"[[x,y],...]\" (base_footprint 기준, 손잡이 꼬리 제외)"),
      BT::InputPort<double>("footprint_padding", 0.05, "몸통 바깥 여유 (global_costmap footprint_padding)"),
      BT::InputPort<int>("confirm_ticks", 2, "막힘이 이 횟수 이어져야 FAILURE"),
      BT::InputPort<std::string>("costmap_topic", std::string("global_costmap/costmap_raw"),
        "nav2_msgs/Costmap 토픽"),
      BT::InputPort<double>("max_costmap_age", 3.0, "costmap 이 이보다 오래되면(s) 막힘으로 본다"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr sub_;
  CostGrid grid_;
  rclcpp::Time grid_stamp_{0, 0, RCL_ROS_TIME};
  bool have_grid_{false};
  std::string body_text_;
  std::vector<Point2> body_;
  BlockedConfirm confirm_;
};

}  // namespace vica_nav2_bt_plugins
