// BT 동작 노드 <StraightPathToGoal goal="{goal}" path="{straight_path}" max_dist="2.0"/>
//
// 사용자 안내 트리(guided.xml)에만 둔다. 로봇이 목적지 max_dist 안이면 로봇 자리에서 목적지 자리까지
// 곧은 경로를 내고 SUCCESS. 밖이면(또는 goal·TF 를 못 읽으면) **아무것도 쓰지 않고** FAILURE —
// 뒤의 planner 갈래가 지금처럼 돈다. max_dist 는 레일 거름망의 handoff_dist_to_goal(2.0)과 같아야
// 한다: 레일을 놓는 순간(목적지 2 m 안)에만 직선을 쓰고, 레일에서 크게 벗어난 회피·레일 서버 이상
// (목적지가 먼데 자유주행으로 떨어진 경우)은 planner 그대로다(test_bt_goal_checker_contract).
// 계산은 점 40개 남짓 채우기뿐이고 부르는 빈도는 planner 와 같다(1 Hz).
#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace vica_nav2_bt_plugins
{

class StraightPathToGoalAction : public BT::SyncActionNode
{
public:
  StraightPathToGoalAction(const std::string & name, const BT::NodeConfiguration & conf);
  StraightPathToGoalAction() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "목적지. 위치만 쓰고 yaw 는 버린다"),
      BT::InputPort<double>("max_dist", 2.0,
        "로봇이 목적지에서 이 거리(m) 안일 때만 직선을 낸다. 레일 handoff_dist_to_goal 과 같게"),
      BT::InputPort<double>("spacing", 0.05, "점 간격(m). global costmap 해상도와 같다"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "경로 frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_footprint"), "로봇 frame"),
      BT::InputPort<double>("transform_tolerance", 0.2, "TF 허용 지연 (s)"),
      BT::OutputPort<nav_msgs::msg::Path>("path", "로봇 -> 목적지 곧은 경로"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
};

}  // namespace vica_nav2_bt_plugins
