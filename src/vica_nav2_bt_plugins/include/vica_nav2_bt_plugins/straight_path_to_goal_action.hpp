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

}
