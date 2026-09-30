// BT 동작 노드 <AlignPathEndToGoal input_path="{path}" goal="{goal}" output_path="{path}"/>
//
// 목적지까지 그린 ComputePathToPose 바로 뒤에 둔다. 경로 끝점이 목적지 max_dist 안이면 끝점
// 방향만 목적지 방향으로 바꾼다. 항상 SUCCESS — 트리의 성공·실패 흐름을 바꾸지 않는다.
// 계산은 점 하나 복사뿐이고 부르는 빈도는 planner 와 같다(1 Hz).
#pragma once

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace vica_nav2_bt_plugins
{

class AlignPathEndToGoalAction : public BT::SyncActionNode
{
public:
  AlignPathEndToGoalAction(const std::string & name, const BT::NodeConfiguration & conf);
  AlignPathEndToGoalAction() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "목적지까지 그린 경로"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "목적지"),
      BT::InputPort<double>("max_dist", 0.5,
        "끝점이 목적지에서 이 거리(m) 안일 때만 바꾼다. goal checker unlatch_distance 와 같다"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "끝점 방향을 맞춘 경로"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace vica_nav2_bt_plugins
