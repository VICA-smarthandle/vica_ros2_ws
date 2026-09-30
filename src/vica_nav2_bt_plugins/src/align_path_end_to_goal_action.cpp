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
    // 입력이 없으면 아무것도 안 하고 지나간다(경로는 앞 노드가 쓴 그대로).
    return BT::NodeStatus::SUCCESS;
  }
  double max_dist = 0.5;
  getInput("max_dist", max_dist);
  if (alignPathEndToGoal(path, goal, max_dist)) {
    // 실주행 뒤 bag(/rosout)에서 몇 번 고쳤는지 셀 수 있게 남긴다. 1 Hz 라 조르지 않는다.
    RCLCPP_INFO(node_->get_logger(), "[AlignPathEndToGoal] 경로 끝 방향을 목적지 방향으로 맞췄다");
  }
  setOutput("output_path", path);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace vica_nav2_bt_plugins

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<vica_nav2_bt_plugins::AlignPathEndToGoalAction>("AlignPathEndToGoal");
}
