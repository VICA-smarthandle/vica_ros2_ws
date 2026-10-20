#include "vica_nav2_bt_plugins/align_path_end_to_goal_action.hpp"

#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "vica_nav2_bt_plugins/path_end_align.hpp"

namespace vica_nav2_bt_plugins
{

AlignPathEndToGoalAction::AlignPathEndToGoalAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus AlignPathEndToGoalAction::tick()
{
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("input_path", path) || !getInput("goal", goal)) {
    return BT::NodeStatus::SUCCESS;
  }
  double max_dist = 0.5;
  getInput("max_dist", max_dist);
  if (alignPathEndToGoal(path, goal, max_dist)) {
    RCLCPP_INFO(node_->get_logger(), "[AlignPathEndToGoal] 경로 끝 방향을 목적지 방향으로 맞췄다");
  }
  setOutput("output_path", path);
  return BT::NodeStatus::SUCCESS;
}

}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<vica_nav2_bt_plugins::AlignPathEndToGoalAction>("AlignPathEndToGoal");
}
