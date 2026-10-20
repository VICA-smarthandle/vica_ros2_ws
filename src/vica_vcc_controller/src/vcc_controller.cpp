#include "vica_vcc_controller/vcc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace vica_vcc_controller
{
namespace
{
core::Pose2D toPose2D(const geometry_msgs::msg::Pose & p)
{
  return {p.position.x, p.position.y, tf2::getYaw(p.orientation)};
}
}

double VccController::steadyNow() const
{
  return time_source_ ? time_source_() : steady_.now().seconds();
}

void VccController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  if (!node) {throw nav2_core::PlannerException("vcc: unable to lock node");}
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  auto dp = [&](const std::string & n, auto def) {
      declare_parameter_if_not_declared(node, name_ + "." + n, rclcpp::ParameterValue(def));
      decltype(def) v = def;
      node->get_parameter(name_ + "." + n, v);
      return v;
    };

  core::PathWindowParams wp;
  wp.max_robot_pose_search_dist = dp("max_robot_pose_search_dist", 3.0);
  wp.reject_unit_path = dp("reject_unit_path", false);
  path_window_ = core::PathWindow(wp);

  core::CoreParams p;
  p.speed.desired = base_speed_ = speed_cap_ = dp("desired_linear_vel", 0.5);
  p.output.max_v = p.speed.desired;
  p.lookahead.time = dp("lookahead_time", 2.5);
  p.lookahead.min_dist = dp("min_lookahead_dist", 0.6);
  p.lookahead.max_dist = dp("max_lookahead_dist", 1.2);
  transform_tolerance_ = dp("transform_tolerance", 0.2);
  p.output.max_w = dp("max_angular_vel", 0.5);
  p.output.max_ang_accel = dp("max_angular_accel", 1.2);
  p.output.max_decel = dp("max_linear_decel", 1.25);
  p.output.planned_decel = dp("planned_linear_decel", p.output.max_decel);
  p.output.ramp_v1 = dp("start_ramp_speed", 0.25);
  p.output.ramp_a1 = dp("start_ramp_accel", 0.5);
  p.output.accel = dp("linear_accel", 0.143);
  p.output.resync_margin = dp("resync_margin", 0.15);
  p.speed.min_speed = dp("min_speed", 0.12);
  p.speed.curve_min_radius = dp("curve_min_radius", 1.2);
  p.speed.curve_decel = dp("curve_decel", 0.3);
  p.speed.preview_dist = dp("preview_dist", 1.5);
  p.speed.slow_clearance = dp("slow_clearance", 0.35);
  p.speed.approach_dist = dp("approach_velocity_scaling_dist", 0.6);
  p.speed.approach_min = dp("min_approach_linear_velocity", 0.05);
  p.lane.max_offset = dp("lane_max_offset", 0.6);
  p.lane.step = dp("lane_step", 0.1);
  p.lane.shift_speeds = dp("lane_shift_speeds", std::vector<double>{0.3, 0.2, 0.1});
  p.lane.horizon = dp("avoid_horizon", 1.5);
  p.lane.target_clearance = dp("target_clearance", 0.20);
  p.lane.w_clear = dp("w_clear", 10.0);
  p.lane.w_rail = dp("w_rail", 1.0);
  p.lane.w_change = dp("w_change", 0.5);
  p.lane.switch_margin = dp("switch_margin", 0.05);
  p.lane.persist_cycles = dp("switch_persist_cycles", 3);
  p.lane.lane_rate = dp("lane_rate", 0.10);
  p.lane.return_clear_time = dp("return_clear_time", 1.0);
  p.resync_offset = dp("lane_resync_threshold", 0.15);
  p.state.turn_enter_angle = dp("turn_enter_angle", 1.047);
  p.state.turn_exit_angle = dp("turn_exit_angle", 0.436);
  p.state.pivot_start_angle = dp("pivot_start_angle", 0.611);
  p.state.min_state_time = dp("min_state_time", 0.5);
  p.state.align_exit_dist = dp("align_exit_dist", 0.5);
  p.arrive_margin = dp("arrive_margin", 0.03);
  p.align_rearm_time = dp("align_rearm_time", 3.0);
  p.turn_enter_persist = dp("turn_enter_persist", 0.0);
  p.turn_lock_target = dp("turn_lock_target", false);
  p.turn_retarget_angle = dp("turn_retarget_angle", M_PI / 2.0);
  p.turn_retarget_persist = dp("turn_retarget_persist", 0.5);
  p.turn_max_rotation = dp("turn_max_rotation", 0.0);
  p.end_extend = dp("end_extend", false);
  p.end_extend_max_lateral = dp("end_extend_max_lateral", 0.08);
  p.end_extend_min_length = dp("end_extend_min_length", 0.3);
  p.pass_arrival = dp("pass_arrival", false);
  p.position_only_yaw_tol = dp("position_only_yaw_tol", 3.0);
  p.new_path_confirm = dp("new_path_collision_confirm", false);
  p.end_straight_lateral = dp("end_straight_lateral", 0.0);
  p.turn.radii = dp("turn_radii", std::vector<double>{0.2, 0.1});
  p.turn.clearance = dp("turn_clearance", 0.05);
  p.turn.keep_clearance = dp("turn_keep_clearance", -1.0);
  p.turn.arc_w = dp("turn_angular_vel", 0.45);
  p.turn.pivot_w = dp("pivot_angular_vel", 0.35);
  p.align.w_max = dp("align_angular_vel", 0.35);
  p.align.alpha = p.output.max_ang_accel;
  p.align.motor_lag = dp("motor_lag", 0.35);
  p.align.settle = dp("align_settle", 0.3);
  p.align.max_attempts = dp("align_max_attempts", 3);
  clearance_window_ = dp("clearance_window", 5.0);
  reset_gap_ = dp("reset_gap", 1.5);
  us_max_age_ = dp("us_max_age", 1.0);
  us_confirm_count_ = dp("us_confirm_count", 2);
  us_confirm_tol_ = dp("us_confirm_tol", 0.15);
  us_arc_points_ = dp("us_arc_points", 7);
  core::UltrasonicParams up;
  up.max_age = us_max_age_;
  up.confirm_count = us_confirm_count_;
  up.confirm_tol = us_confirm_tol_;
  up.near_confirm_range = dp("us_near_confirm_range", 0.40);
  up.memory_time = dp("us_memory_time", 1.5);
  up.pass_memory = dp("us_pass_memory", false);
  up.pass_margin = dp("us_pass_margin", 0.2);
  up.pass_max_dist = dp("us_pass_max_dist", 2.0);
  up.memory_slots = dp("us_memory_slots", 4);
  publish_state_ = dp("publish_state", true);
  const std::vector<std::string> topics = dp(
    "ultrasonic_topics", std::vector<std::string>{
      "/ultrasonic/front_left", "/ultrasonic/front_right",
      "/ultrasonic/left_wheel", "/ultrasonic/right_wheel"});

  for (const auto & pt : costmap_ros_->getRobotFootprint()) {p.footprint.push_back({pt.x, pt.y});}
  params_ = p;
  us_rear_x_ = 0.0;
  for (const auto & q : params_.footprint) {us_rear_x_ = std::min(us_rear_x_, q.x);}
  core_.configure(params_);
  field_.setFootprint(params_.footprint);

  global_frame_ = costmap_ros_->getGlobalFrameID();
  us_max_range_ = dp("us_max_range_per_topic", std::vector<double>{});
  us_max_range_.resize(topics.size(), 0.0);
  us_channels_.assign(topics.size(), core::UltrasonicChannel(up));
  us_subs_.clear();
  for (size_t i = 0; i < topics.size(); ++i) {
    us_subs_.push_back(node->create_subscription<sensor_msgs::msg::Range>(
        topics[i], rclcpp::SensorDataQoS(),
        [this, i](sensor_msgs::msg::Range::ConstSharedPtr m) {
          core::RangeReading r;
          r.range = m->range;
          r.min_range = m->min_range;
          r.max_range = m->max_range;
          r.fov = m->field_of_view;
          r.recv_time = steadyNow();
          if (m->header.frame_id.empty()) {return;}
          try {
            const auto t = tf_->lookupTransform(global_frame_, m->header.frame_id, tf2::TimePointZero);
            r.sensor = {t.transform.translation.x, t.transform.translation.y,
              tf2::getYaw(t.transform.rotation)};
          } catch (tf2::TransformException &) {
            return;
          }
          std::lock_guard<std::mutex> lock(us_mutex_);
          us_channels_[i].push(core::capRange(r, us_max_range_[i]));
        }));
  }

  state_pub_ = node->create_publisher<std_msgs::msg::String>("vcc/state", 10);
  lane_pub_ = node->create_publisher<nav_msgs::msg::Path>("vcc/lane_plan", 1);
  RCLCPP_INFO(logger_, "VCC configured: %zu ultrasonic topics, footprint %zu points",
    topics.size(), params_.footprint.size());
}

void VccController::cleanup()
{
  state_pub_.reset();
  lane_pub_.reset();
  us_subs_.clear();
  core_.reset();
}

void VccController::activate()
{
  state_pub_->on_activate();
  lane_pub_->on_activate();
  core_.reset();
  last_compute_ = -1.0;
}

void VccController::deactivate()
{
  state_pub_->on_deactivate();
  lane_pub_->on_deactivate();
  core_.reset();
  path_window_.setPlan({});
  last_goal_x_ = last_goal_y_ = 1e9;
}

void VccController::setPlan(const nav_msgs::msg::Path & path)
{
  core::Path plan;
  for (const auto & ps : path.poses) {plan.push_back(toPose2D(ps.pose));}
  path_window_.setPlan(std::move(plan));
  plan_frame_ = path.header.frame_id;
  core_.onNewPath();
  if (path.poses.empty()) {return;}
  const auto & e = path.poses.back().pose.position;
  if (std::hypot(e.x - last_goal_x_, e.y - last_goal_y_) > 0.5) {core_.onNewGoal();}
  last_goal_x_ = e.x;
  last_goal_y_ = e.y;
}

void VccController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    speed_cap_ = base_speed_;
  } else if (percentage) {
    speed_cap_ = base_speed_ * speed_limit / 100.0;
  } else {
    speed_cap_ = speed_limit;
  }
}

bool VccController::transformPose(
  const std::string & frame, const geometry_msgs::msg::PoseStamped & in,
  geometry_msgs::msg::PoseStamped & out) const
{
  if (in.header.frame_id == frame) {out = in; return true;}
  try {
    tf_->transform(in, out, frame, tf2::durationFromSec(transform_tolerance_));
    out.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

core::Path VccController::windowPlan(
  const geometry_msgs::msg::PoseStamped & pose, core::Pose2D & goal_robot)
{
  if (path_window_.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(plan_frame_, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }
  const core::Pose2D robot = toPose2D(robot_pose.pose);
  plan_robot_yaw_ = robot.yaw;
  const double max_extent =
    std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY()) / 2.0;

  core::Path window;
  switch (path_window_.window(robot, max_extent, window)) {
    case core::WindowStatus::EmptyPlan:
      throw nav2_core::PlannerException("Received plan with zero length");
    case core::WindowStatus::UnitPath:
      throw nav2_core::PlannerException("vcc: plan with length of one (reject_unit_path)");
    case core::WindowStatus::NoPosesInWindow:
      throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
    case core::WindowStatus::Ok:
      break;
  }
  core::Path out;
  out.reserve(window.size());
  for (const auto & p : window) {out.push_back(core::toRobotFrame(robot, p));}
  goal_robot = core::toRobotFrame(robot, path_window_.plan().back());
  return out;
}

void VccController::fillClearance(const geometry_msgs::msg::PoseStamped & pose)
{
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  const double res = costmap_->getResolution();
  const int n = static_cast<int>(std::round(clearance_window_ / res));
  const double rx = pose.pose.position.x, ry = pose.pose.position.y;
  int mx0, my0;
  costmap_->worldToMapEnforceBounds(rx - clearance_window_ / 2.0, ry - clearance_window_ / 2.0, mx0, my0);
  double ox, oy;
  costmap_->mapToWorld(mx0, my0, ox, oy);
  auto & g = field_.grid();
  g.reset(ox - res / 2.0, oy - res / 2.0, res, n, n);
  const unsigned int sx = costmap_->getSizeInCellsX(), sy = costmap_->getSizeInCellsY();
  for (int iy = 0; iy < n; ++iy) {
    for (int ix = 0; ix < n; ++ix) {
      const unsigned int mx = mx0 + ix, my = my0 + iy;
      if (mx >= sx || my >= sy) {continue;}
      if (costmap_->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {g.markLethal(ix, iy);}
    }
  }
  g.compute();
}

void VccController::fillUltrasonic(double now, const core::Pose2D & robot)
{
  std::vector<core::Point2D> pts;
  std::lock_guard<std::mutex> lock(us_mutex_);
  us_fresh_ = 0;
  us_memos_ = 0;
  for (auto & ch : us_channels_) {
    ch.forget(robot, us_rear_x_);
    us_memos_ += static_cast<int>(ch.memoryCount());
    if (ch.fresh(now, us_max_age_)) {++us_fresh_;}
    for (const auto & r : ch.obstacles(now)) {
      for (const auto & q : core::readingToArcPoints(r, us_arc_points_)) {pts.push_back(q);}
    }
  }
  field_.setPoints(std::move(pts));
}

geometry_msgs::msg::TwistStamped VccController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const double now = steadyNow();
  const double dt = last_compute_ < 0.0 ? 0.1 : std::clamp(now - last_compute_, 0.02, 0.2);
  if (last_compute_ >= 0.0 && now - last_compute_ > reset_gap_) {
    core_.reset({velocity.linear.x, velocity.angular.z});
  }
  last_compute_ = now;

  double xy_tol = 0.25, yaw_tol = 0.25, rot_stopped = 0.05;
  if (goal_checker) {
    geometry_msgs::msg::Pose pt;
    geometry_msgs::msg::Twist vt;
    if (goal_checker->getTolerances(pt, vt)) {
      xy_tol = pt.position.x;
      yaw_tol = tf2::getYaw(pt.orientation);
      if (vt.angular.z > 0.0) {rot_stopped = vt.angular.z;}
    }
  }

  core::Pose2D goal_robot;
  core::Path robot_path = windowPlan(pose, goal_robot);

  fillClearance(pose);
  fillUltrasonic(now, toPose2D(pose.pose));

  core::CoreInputs in;
  in.now = now;
  in.dt = dt;
  in.path = std::move(robot_path);
  in.goal = goal_robot;
  in.measured = {velocity.linear.x, velocity.angular.z};
  in.xy_tol = xy_tol;
  in.yaw_tol = yaw_tol;
  in.rot_stopped = rot_stopped;
  in.speed_cap = speed_cap_;
  in.robot_yaw = plan_robot_yaw_;
  const core::Pose2D robot = toPose2D(pose.pose);
  in.clearance = [this, robot](const core::Pose2D & p) {
      return field_.clearance(core::toParent(robot, p));
    };

  const core::CoreOutput out = core_.step(in);

  if (publish_state_ && state_pub_->is_activated()) {
    std_msgs::msg::String s;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "state=%s reason=%s offset=%.2f target=%.2f blocked=%d turn=%d align=%d fail=%d v=%.3f w=%.3f "
      "us_fresh=%d rot=%.0f ext=%d defer=%d us_mem=%d str=%d",
      core::stateName(out.state), out.reason, out.offset, out.target, out.lanes_blocked ? 1 : 0,
      static_cast<int>(out.turn_mode), out.align_attempts, static_cast<int>(out.failure),
      out.cmd.v, out.cmd.w, us_fresh_, out.turn_rotated * 180.0 / M_PI, out.end_extended ? 1 : 0,
      out.collision_deferred ? 1 : 0, us_memos_, out.end_straight ? 1 : 0);
    s.data = buf;
    state_pub_->publish(s);
    nav_msgs::msg::Path lp;
    lp.header.frame_id = costmap_ros_->getBaseFrameID();
    lp.header.stamp = pose.header.stamp;
    for (const auto & q : out.lane_path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = lp.header;
      ps.pose.position.x = q.x;
      ps.pose.position.y = q.y;
      ps.pose.orientation.z = std::sin(q.yaw / 2.0);
      ps.pose.orientation.w = std::cos(q.yaw / 2.0);
      lp.poses.push_back(ps);
    }
    lane_pub_->publish(lp);
  }

  switch (out.failure) {
    case core::Failure::CollisionAhead:
      throw nav2_core::PlannerException("vcc: collision ahead");
    case core::Failure::Blocked:
      throw nav2_core::PlannerException("vcc: blocked");
    case core::Failure::AlignFailed:
      throw nav2_core::PlannerException("vcc: align failed");
    case core::Failure::None:
      break;
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = pose.header.frame_id;
  cmd.header.stamp = clock_->now();
  cmd.twist.linear.x = out.cmd.v;
  cmd.twist.angular.z = out.cmd.w;
  return cmd;
}
}

PLUGINLIB_EXPORT_CLASS(vica_vcc_controller::VccController, nav2_core::Controller)
