#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "vica_nav2_bt_plugins/route_path_check.hpp"

namespace vica_nav2_bt_plugins
{

class IsRoutePathUsableCondition : public BT::ConditionNode
{
public:
  IsRoutePathUsableCondition(const std::string & name, const BT::NodeConfiguration & conf);
  IsRoutePathUsableCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "레일 경로 (ComputeRoute 의 출력)"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "목적지. 인계 판정에 쓴다"),
      BT::InputPort<double>("handoff_dist_to_goal", 2.0,
        "목적지에서 이 거리(m) 안이면 레일을 버리고 자유주행으로. 0 이면 끔"),
      BT::InputPort<int>("min_poses", 2, "점이 이보다 적으면 못 쓴다"),
      BT::InputPort<double>("max_dist_from_path", 1.5,
        "로봇이 경로에서 이보다 멀면 못 쓴다 (m). 지역 costmap 반폭(3 m)보다 작게"),
      BT::InputPort<double>("rejoin_dist_from_path", -1.0,
        "한 번 max_dist 밖으로 나가면 이 거리(m) 안으로 들어와야 다시 쓴다. 0 이하면 끔(문턱 하나)"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "경로 frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_footprint"), "로봇 frame"),
      BT::InputPort<double>("transform_tolerance", 0.2, "TF 허용 지연 (s)"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  RailDistanceGate gate_;
};

}
