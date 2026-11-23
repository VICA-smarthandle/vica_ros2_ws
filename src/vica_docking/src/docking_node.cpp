#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "vica_interfaces/srv/save_home.hpp"

#include "vica_docking/approach.hpp"
#include "vica_docking/dock_spot.hpp"
#include "vica_docking/undock.hpp"
#include "vica_docking/v_detector.hpp"

namespace vd = vica_docking;
using namespace std::chrono_literals;

namespace
{

double mono_now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string fmt(const char * f, double v)
{
  char b[64];
  std::snprintf(b, sizeof(b), f, v);
  return b;
}

std::string jstr(const std::string & s)
{
  std::string o = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\') {
      o += '\\';
    }
    o += c;
  }
  return o + "\"";
}

std::string now_text()
{
  std::time_t t = std::time(nullptr);
  char b[32];
  std::strftime(b, sizeof(b), "%Y-%m-%dT%H:%M:%S", std::localtime(&t));
  return b;
}

double quat_yaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}

class DockingNode : public rclcpp::Node
{
public:
  DockingNode()
  : rclcpp::Node("vica_docking")
  {
    std::string dest = declare_parameter<std::string>("destinations_path", "");
    if (dest.empty()) {
      dest = std::string(std::getenv("HOME") ? std::getenv("HOME") : "") +
        "/vica_data/destinations/default/destinations.yaml";
    } else if (dest[0] == '~') {
      dest = std::string(std::getenv("HOME") ? std::getenv("HOME") : "") + dest.substr(1);
    }
    const std::filesystem::path dp(dest);
    dock_yaml_ = (dp.parent_path() / "dock.yaml").string();
    map_id_ = declare_parameter<std::string>("map_id", "");
    if (map_id_.empty()) {
      map_id_ = dp.parent_path().filename().string();
    }
    auto_initial_pose_ = declare_parameter<bool>("auto_initial_pose", true);
    boot_check_s_ = declare_parameter<double>("boot_check_s", 20.0);
    trace_dir_ = declare_parameter<std::string>("trace_dir", "");

    ap_.target_cm = declare_parameter<double>("approach.target_cm", ap_.target_cm);
    ap_.stage_cm = declare_parameter<double>("approach.stage_cm", ap_.stage_cm);
    ap_.lead = declare_parameter<double>("approach.lead", ap_.lead);
    ap_.vmax = declare_parameter<double>("approach.vmax", ap_.vmax);
    ap_.vfinal = declare_parameter<double>("approach.vfinal", ap_.vfinal);
    ap_.max_time = declare_parameter<double>("approach.max_time", ap_.max_time);
    ap_.lost_timeout_s = declare_parameter<double>("approach.lost_timeout_s", ap_.lost_timeout_s);
    up_.until_cm = declare_parameter<double>("undock.until_cm", up_.until_cm);
    up_.clear_pre = declare_parameter<double>("undock.clear_pre", up_.clear_pre);
    up_.turn_done_deg = declare_parameter<double>("undock.turn_done_deg", up_.turn_done_deg);
    up_.lead_back = declare_parameter<double>("undock.lead_back", up_.lead_back);
    up_.lead_turn_deg = declare_parameter<double>("undock.lead_turn_deg", up_.lead_turn_deg);
    up_.turn_dir = static_cast<int>(declare_parameter<int>("undock.turn_dir", up_.turn_dir));
    up_.wait_timeout = declare_parameter<double>("undock.wait_timeout", up_.wait_timeout);

    srv_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    main_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_req", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(
      "/vica/dock/state", rclcpp::QoS(1).transient_local().reliable());
    init_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    using Trig = std_srvs::srv::Trigger;
    auto mk = [this](const char * name, auto fn) {
        services_.push_back(create_service<Trig>(
          name, [this, fn](const std::shared_ptr<Trig::Request>, std::shared_ptr<Trig::Response> res) {
            (this->*fn)(*res);
          }, rmw_qos_profile_services_default, srv_group_));
      };
    set_srv_ = create_service<vica_interfaces::srv::SaveHome>(
      "/vica/dock/set",
      [this](
        const std::shared_ptr<vica_interfaces::srv::SaveHome::Request> req,
        std::shared_ptr<vica_interfaces::srv::SaveHome::Response> res) {srv_set(*req, *res);},
      rmw_qos_profile_services_default, srv_group_);
    mk("/vica/dock/start", &DockingNode::srv_start);
    mk("/vica/dock/undock", &DockingNode::srv_undock);
    mk("/vica/dock/cancel", &DockingNode::srv_cancel);
    mk("/vica/dock/register", &DockingNode::srv_register);
    mk("/vica/dock/get", &DockingNode::srv_get);
    mk("/vica/dock/delete", &DockingNode::srv_delete);

    RCLCPP_INFO(
      get_logger(), "도크 위치 파일 %s (지도 %s) · 평소 구독 0개", dock_yaml_.c_str(), map_id_.c_str());
    {
      std::lock_guard<std::mutex> lk(m_);
      publish_state_locked();
      if (auto_initial_pose_) {
        start_job_locked(Job::kBoot);
      }
    }
  }

private:
  enum class Job { kNone, kBoot, kRegister, kApproach, kRetreat, kUndock };

  static const char * job_name(Job j)
  {
    switch (j) {
      case Job::kNone: return "";
      case Job::kBoot: return "boot_check";
      case Job::kRegister: return "register";
      case Job::kApproach: return "approach";
      case Job::kRetreat: return "retreat";
      case Job::kUndock: return "undock";
    }
    return "";
  }

  bool motion_job(Job j) const {return j == Job::kApproach || j == Job::kRetreat || j == Job::kUndock;}

  void start_job_locked(Job j)
  {
    job_ = j;
    job_t0_ = mono_now();
    holding_ = false;
    find_streak_ = 0;
    find_det_.reset();
    finder_ = std::make_unique<vd::VDetector>();
    auto opt = rclcpp::SubscriptionOptions();
    opt.callback_group = main_group_;
    if (!scan_sub_) {
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {on_scan(m);}, opt);
    }
    if (motion_job(j)) {
      if (!estop_sub_) {
        estop_sub_ = create_subscription<std_msgs::msg::Bool>(
          "/estop_state", 10, [this](std_msgs::msg::Bool::ConstSharedPtr m) {on_estop(m);}, opt);
      }
      if (!cmd_watch_sub_) {
        auto o2 = opt;
        o2.ignore_local_publications = true;
        cmd_watch_sub_ = create_subscription<geometry_msgs::msg::Twist>(
          "/cmd_vel_req", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {on_foreign_cmd(m);},
          o2);
      }
    }
    if (j == Job::kRetreat || j == Job::kUndock) {
      if (!imu_sub_) {
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          "/imu/base_link", rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {on_imu(m);}, opt);
      }
      static const char * kUs[4] = {
        "/ultrasonic/dock_rear_right", "/ultrasonic/dock_rear_left",
        "/ultrasonic/dock_right_rear_wheel", "/ultrasonic/dock_left_rear_wheel"};
      if (us_subs_.empty()) {
        for (int i = 0; i < 4; ++i) {
          us_subs_.push_back(create_subscription<sensor_msgs::msg::Range>(
            kUs[i], rclcpp::SensorDataQoS(),
            [this, i](sensor_msgs::msg::Range::ConstSharedPtr m) {on_us(i, m);}, opt));
        }
      }
    }
    if (j == Job::kRegister) {
      ensure_amcl_sub_locked();
    }
    tail_ticks_ = 0;
    if (!job_timer_) {
      job_timer_ = create_wall_timer(50ms, [this]() {on_job_timer();}, main_group_);
    }
    open_trace_locked();
    RCLCPP_INFO(get_logger(), "시작: %s", job_name(j));
    publish_state_locked();
  }

  void end_job_locked()
  {
    const bool moved = motion_job(job_);
    RCLCPP_INFO(get_logger(), "끝: %s · %s %s", job_name(job_), last_result_.c_str(), last_reason_.c_str());
    job_ = Job::kNone;
    job_end_t_ = mono_now();
    approach_.reset();
    undock_.reset();
    close_trace_locked();
    tail_ticks_ = moved ? 10 : 0;
    if (!moved) {
      release_subs_locked();
    }
    publish_state_locked();
  }

  void release_subs_locked()
  {
    scan_sub_.reset();
    estop_sub_.reset();
    cmd_watch_sub_.reset();
    imu_sub_.reset();
    us_subs_.clear();
    if (pose_init_ == PoseInit::kNone && job_ != Job::kRegister) {
      amcl_sub_.reset();
    }
    if (job_timer_) {
      job_timer_->cancel();
      job_timer_.reset();
    }
  }

  void srv_start(std_srvs::srv::Trigger::Response & res)
  {
    std::lock_guard<std::mutex> lk(m_);
    if (job_ != Job::kNone && job_ != Job::kBoot) {
      res.success = false;
      res.message = "busy";
      return;
    }
    if (docked_) {
      res.success = false;
      res.message = "already_docked";
      return;
    }
    if (job_ == Job::kBoot) {
      job_ = Job::kNone;
    }
    stop_pose_init_locked("moved");
    cancel_req_ = false;
    attempt_ = 1;
    begin_approach_locked();
    res.success = true;
    res.message = "accepted";
  }

  void srv_undock(std_srvs::srv::Trigger::Response & res)
  {
    std::lock_guard<std::mutex> lk(m_);
    if (job_ != Job::kNone && job_ != Job::kBoot) {
      res.success = false;
      res.message = "busy";
      return;
    }
    if (job_ == Job::kBoot) {
      job_ = Job::kNone;
    }
    stop_pose_init_locked("moved");
    cancel_req_ = false;
    undock_ = std::make_unique<vd::UndockController>(up_);
    resume_turn_wait_ = undock_pending_ && pending_turn_only_;
    if (resume_turn_wait_) {
      amcl_.reset();
      amcl_sub_.reset();
      ensure_amcl_sub_locked();
    } else {
      undock_->start(mono_now());
    }
    last_action_ = "undock";
    last_result_.clear();
    last_reason_.clear();
    last_why_.clear();
    phase_.clear();
    err_x_ = err_y_ = err_th_ = 0.0;
    start_job_locked(Job::kUndock);
    res.success = true;
    res.message = resume_turn_wait_ ? "accepted_resume_turn" : "accepted";
  }

  void srv_cancel(std_srvs::srv::Trigger::Response & res)
  {
    std::lock_guard<std::mutex> lk(m_);
    if (approach_) {
      approach_->abort(vd::Fail::kCanceled, "사용자가 멈춤");
    }
    if (undock_) {
      undock_->abort(vd::Fail::kCanceled, "사용자가 멈춤");
    }
    res.success = motion_job(job_);
    res.message = res.success ? "canceling" : "idle";
    if (res.success) {
      cancel_req_ = true;
    }
    if (!res.success && (undock_pending_ || (near_dock_ && !docked_)) && mono_now() - job_end_t_ > 1.0) {
      undock_pending_ = false;
      if (!docked_) {
        near_dock_ = false;
      }
      res.success = true;
      res.message = "pending_cleared";
      publish_state_locked();
    }
  }

  void srv_register(std_srvs::srv::Trigger::Response & res)
  {
    {
      std::lock_guard<std::mutex> lk(m_);
      if (job_ != Job::kNone && job_ != Job::kBoot) {
        res.success = false;
        res.message = "busy";
        return;
      }
      register_reply_.clear();
      reg_seen_ = 0;
      reg_last_.reset();
      amcl_.reset();
      amcl_sub_.reset();
      start_job_locked(Job::kRegister);
    }
    const double t_end = mono_now() + 4.0;
    while (mono_now() < t_end) {
      {
        std::lock_guard<std::mutex> lk(m_);
        if (!register_reply_.empty()) {
          res.success = register_ok_;
          res.message = register_reply_;
          return;
        }
      }
      std::this_thread::sleep_for(50ms);
    }
    std::lock_guard<std::mutex> lk(m_);
    if (job_ == Job::kRegister) {
      end_job_locked();
    }
    res.success = false;
    res.message = R"({"reason":"not_found"})";
  }

  std::string spot_json(const vd::DockSpot & s) const
  {
    const auto st = vd::staging_pose(s, geom_.bumper_x);
    const auto dk = vd::docked_pose(s, geom_.bumper_x);
    std::ostringstream o;
    o << "{\"exists\":true,\"map_id\":" << jstr(s.map_id)
      << ",\"staging\":{\"x\":" << fmt("%.4f", st.x) << ",\"y\":" << fmt("%.4f", st.y)
      << ",\"yaw\":" << fmt("%.5f", st.yaw) << "}"
      << ",\"docked\":{\"x\":" << fmt("%.4f", dk.x) << ",\"y\":" << fmt("%.4f", dk.y)
      << ",\"yaw\":" << fmt("%.5f", dk.yaw) << "}"
      << ",\"saved_at\":" << jstr(s.saved_at)
      << ",\"source\":" << jstr(s.source)
      << ",\"refined\":" << (s.refined ? "true" : "false")
      << ",\"refined_from\":" << jstr(s.refined_from)
      << ",\"refined_cm\":" << fmt("%.1f", s.refined_cm)
      << ",\"refined_deg\":" << fmt("%.1f", s.refined_deg)
      << ",\"visited_ok\":" << (s.visited_ok ? "true" : "false") << "}";
    return o.str();
  }

  void srv_get(std_srvs::srv::Trigger::Response & res)
  {
    const auto s = vd::read_dock_spot(dock_yaml_);
    if (!s || s->map_id != map_id_) {
      res.success = false;
      res.message = R"({"exists":false})";
      return;
    }
    res.success = true;
    res.message = spot_json(*s);
  }

  void srv_set(
    const vica_interfaces::srv::SaveHome::Request & req, vica_interfaces::srv::SaveHome::Response & res)
  {
    std::lock_guard<std::mutex> lk(m_);
    res.accepted = false;
    res.visited_ok = false;
    if (job_ != Job::kNone && job_ != Job::kBoot) {
      res.message = "busy";
      return;
    }
    if (!req.map_id.empty() && req.map_id != map_id_) {
      res.message = "map_mismatch";
      return;
    }
    if (!std::isfinite(req.x) || !std::isfinite(req.y) || !std::isfinite(req.yaw)) {
      res.message = "bad_pose";
      return;
    }
    vd::DockSpot s;
    s.map_id = map_id_;
    s.apex_x = req.x;
    s.apex_y = req.y;
    s.fwd_yaw = vd::wrap_angle(vd::rad(req.yaw));
    s.stage_cm = ap_.stage_cm;
    s.target_cm = ap_.target_cm;
    s.saved_at = now_text();
    s.source = req.source.empty() ? "map_pick" : req.source;
    s.visited_ok = false;
    std::string err;
    if (!vd::write_dock_spot(dock_yaml_, s, &err)) {
      res.message = "write_failed";
      RCLCPP_WARN(get_logger(), "도크 위치(지도에서 지정) 저장 실패: %s", err.c_str());
      return;
    }
    res.accepted = true;
    res.message = spot_json(s);
    RCLCPP_INFO(
      get_logger(), "도크 위치 저장(%s): 꼭짓점 (%.2f, %.2f) 방향 %.1f°", s.source.c_str(), s.apex_x,
      s.apex_y, req.yaw);
    publish_state_locked();
  }

  void srv_delete(std_srvs::srv::Trigger::Response & res)
  {
    std::error_code ec;
    const bool removed = std::filesystem::remove(dock_yaml_, ec);
    res.success = !ec;
    res.message = removed ? "deleted" : (ec ? ec.message() : "none");
  }

  void on_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr m)
  {
    std::lock_guard<std::mutex> lk(m_);
    const double now = mono_now();
    if (job_ == Job::kApproach && approach_) {
      approach_->on_scan(now, m->ranges, m->angle_min, m->angle_increment, m->range_min);
      trace_locked(now);
    } else if ((job_ == Job::kRetreat || job_ == Job::kUndock) && undock_) {
      undock_->on_scan(now, m->ranges, m->angle_min, m->angle_increment, m->range_min);
      trace_locked(now);
    } else if ((job_ == Job::kBoot || job_ == Job::kRegister) && finder_) {
      const auto det = finder_->detect(m->ranges, m->angle_min, m->angle_increment, m->range_min);
      if (!det) {
        return;
      }
      const auto p = vd::pose_in_dock(*det);
      if (job_ == Job::kRegister) {
        ++reg_seen_;
        reg_last_ = det;
      }
      const bool same = find_det_ && std::abs(p.x - find_pose_.x) < 0.02 &&
        std::abs(p.y - find_pose_.y) < 0.02;
      find_streak_ = same ? find_streak_ + 1 : 1;
      find_det_ = det;
      find_pose_ = p;
    }
  }

  void on_imu(sensor_msgs::msg::Imu::ConstSharedPtr m)
  {
    std::lock_guard<std::mutex> lk(m_);
    if (undock_) {
      undock_->on_gyro(mono_now(), m->angular_velocity.z);
    }
  }

  void on_us(int i, sensor_msgs::msg::Range::ConstSharedPtr m)
  {
    std::lock_guard<std::mutex> lk(m_);
    if (undock_) {
      undock_->on_ultrasonic(i, mono_now(), m->range, m->max_range);
    }
  }

  void on_estop(std_msgs::msg::Bool::ConstSharedPtr m)
  {
    if (!m->data) {
      return;
    }
    std::lock_guard<std::mutex> lk(m_);
    if (approach_) {
      approach_->abort(vd::Fail::kEstop, "비상정지");
    }
    if (undock_) {
      undock_->abort(vd::Fail::kEstop, "비상정지");
    }
  }

  void on_foreign_cmd(geometry_msgs::msg::Twist::ConstSharedPtr m)
  {
    if (std::abs(m->linear.x) < 1e-3 && std::abs(m->angular.z) < 1e-3) {
      return;
    }
    std::lock_guard<std::mutex> lk(m_);
    if (approach_) {
      approach_->abort(vd::Fail::kConflict, "다른 노드가 /cmd_vel_req 를 냄");
    }
    if (undock_) {
      undock_->abort(vd::Fail::kConflict, "다른 노드가 /cmd_vel_req 를 냄");
    }
  }

  void on_amcl(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m)
  {
    std::lock_guard<std::mutex> lk(m_);
    amcl_ = vd::Pose2{m->pose.pose.position.x, m->pose.pose.position.y, quat_yaw(m->pose.pose.orientation)};
    amcl_t_ = mono_now();
  }

  void on_job_timer()
  {
    std::lock_guard<std::mutex> lk(m_);
    const double now = mono_now();
    if (job_ == Job::kNone) {
      if (tail_ticks_ > 0) {
        cmd_pub_->publish(geometry_msgs::msg::Twist());
        if (--tail_ticks_ == 0) {
          release_subs_locked();
        }
      }
      return;
    }
    if (job_ == Job::kBoot) {
      boot_tick_locked(now);
    } else if (job_ == Job::kRegister) {
      register_tick_locked(now);
    } else if (job_ == Job::kApproach && approach_) {
      const vd::Cmd c = approach_->on_timer(now);
      publish_cmd(c);
      const std::string ph = vd::phase_name(approach_->phase());
      if (ph != phase_) {
        phase_ = ph;
        publish_state_locked();
      }
      if (approach_->finished()) {
        approach_done_locked();
      }
    } else if ((job_ == Job::kRetreat || job_ == Job::kUndock) && undock_) {
      if (resume_turn_wait_ && !undock_->finished()) {
        if (!amcl_ && now - job_t0_ < 1.0) {
          publish_cmd({});
          return;
        }
        const double remaining = resume_remaining_deg_locked();
        RCLCPP_INFO(get_logger(), "나오기 이어서: 남은 회전 %+.1f° 만", remaining);
        undock_->start_turn_only(now, remaining);
      }
      resume_turn_wait_ = false;
      const vd::Cmd c = undock_->on_timer(now);
      publish_cmd(c);
      const std::string ph = vd::phase_name(undock_->phase());
      const bool hold = undock_->holding();
      if (ph != phase_ || hold != holding_) {
        phase_ = ph;
        holding_ = hold;
        publish_state_locked();
      }
      if (undock_->finished()) {
        if (job_ == Job::kRetreat) {
          retreat_done_locked();
        } else {
          undock_done_locked();
        }
      }
    }
  }

  void publish_cmd(const vd::Cmd & c)
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = c.v;
    t.angular.z = c.w;
    cmd_pub_->publish(t);
  }

  void begin_approach_locked()
  {
    undock_pending_ = false;
    last_refined_ = false;
    {
      const auto s = vd::read_dock_spot(dock_yaml_);
      if (s && s->map_id == map_id_ && !s->visited_ok) {
        amcl_.reset();
        amcl_sub_.reset();
        ensure_amcl_sub_locked();
      }
    }
    approach_ = std::make_unique<vd::ApproachController>(ap_);
    approach_->start(mono_now());
    last_action_ = "dock";
    last_result_.clear();
    last_reason_.clear();
    last_why_.clear();
    phase_.clear();
    start_job_locked(Job::kApproach);
  }

  void approach_done_locked()
  {
    const auto r = approach_->result();
    const auto lp = approach_->last_pose();
    err_x_ = r.err_x_cm;
    err_y_ = r.err_y_cm;
    err_th_ = r.err_th_deg;
    if (r.success) {
      docked_ = true;
      near_dock_ = true;
      last_result_ = "success";
      last_reason_.clear();
      last_why_.clear();
      RCLCPP_INFO(
        get_logger(), "도킹 끝: 앞뒤 %+.2f cm 좌우 %+.2f cm 방향 %+.2f° (시도 %d)", r.err_x_cm,
        r.err_y_cm, r.err_th_deg, attempt_);
      if (!refine_spot_locked(r.final_pose, true)) {
        const auto s0 = vd::read_dock_spot(dock_yaml_);
        const bool coarse = s0 && s0->map_id == map_id_ && !s0->visited_ok && s0->source == "map_pick";
        if (!coarse) {
          mark_visited_locked();
          begin_pose_init_locked(PoseInit::kCheck, r.final_pose);
        } else {
          RCLCPP_WARN(get_logger(), "도크 위치(지도에서 지정)를 못 고쳤다 — AMCL 값 없음. 다음 도킹 때 다시 고친다");
        }
      }
      end_job_locked();
      return;
    }
    if (r.fail == vd::Fail::kOutOfRange && r.has_start) {
      refine_spot_locked(r.start, false);
    }
    const bool near_fail = r.fail == vd::Fail::kLost || r.fail == vd::Fail::kMisaligned ||
      r.fail == vd::Fail::kTooClose;
    const double d_stage = geom_.bumper_x + ap_.stage_cm / 100.0;
    if (near_fail && attempt_ == 1 && !cancel_req_ && lp && lp->x < d_stage - 0.05) {
      RCLCPP_WARN(get_logger(), "도킹 1차 실패(%s) — 준비 위치로 물러나 다시", vd::fail_name(r.fail));
      near_dock_ = (lp->x - geom_.bumper_x) < up_.until_cm / 100.0;
      attempt_ = 2;
      vd::UndockParams rp = up_;
      rp.until_cm = ap_.stage_cm;
      rp.turn_deg = 0.0;
      undock_ = std::make_unique<vd::UndockController>(rp);
      undock_->start(mono_now());
      approach_.reset();
      job_ = Job::kRetreat;
      start_job_locked(Job::kRetreat);
      return;
    }
    docked_ = false;
    if (lp) {
      near_dock_ = (lp->x - geom_.bumper_x) < up_.until_cm / 100.0;
    }
    fail_locked(r.fail, r.why);
  }

  void retreat_done_locked()
  {
    const auto r = undock_->result();
    if (r.success) {
      undock_.reset();
      begin_approach_locked();
      return;
    }
    const auto lx = undock_->last_x();
    if (lx) {
      near_dock_ = (*lx - geom_.bumper_x) < up_.until_cm / 100.0;
    }
    fail_locked(r.fail, r.why);
  }

  static bool unseen_means_away(vd::Fail f, int scans)
  {
    return f == vd::Fail::kNotFound && scans >= 10;
  }

  void undock_done_locked()
  {
    const auto r = undock_->result();
    if (r.success) {
      docked_ = false;
      near_dock_ = false;
      undock_pending_ = false;
      last_result_ = "success";
      RCLCPP_INFO(
        get_logger(), "해제 끝: 후진 %.1f cm(넘침 %.1f) 회전 %.1f°", r.back_cm, r.back_overshoot_cm,
        r.turned_deg);
      end_job_locked();
      return;
    }
    const auto ph = undock_->abort_phase();
    const bool turning = ph == vd::UndockPhase::kSettleBack || ph == vd::UndockPhase::kTurn ||
      ph == vd::UndockPhase::kSettleTurn;
    if (turning && std::abs(undock_->turn_remaining_deg()) <= up_.turn_done_deg) {
      docked_ = false;
      near_dock_ = false;
      undock_pending_ = false;
      last_result_ = "success";
      RCLCPP_INFO(
        get_logger(), "해제 끝: 남은 회전 %.1f° — 끝난 것으로 본다(멈춘 사유 %s)",
        undock_->turn_remaining_deg(), vd::fail_name(r.fail));
      end_job_locked();
      return;
    }
    if (ph == vd::UndockPhase::kBack || turning) {
      undock_pending_ = true;
      pending_turn_only_ = turning;
      pending_remaining_deg_ = undock_->turn_remaining_deg();
      RCLCPP_INFO(
        get_logger(), "나오다 멈춤(%s): %s", vd::phase_name(ph),
        turning ? "남은 회전만 이어서 할 수 있다" : "후진부터 이어서 할 수 있다");
    }
    if (turning) {
      near_dock_ = false;
      docked_ = false;
      fail_locked(r.fail, r.why);
      return;
    }
    const auto lx = undock_->last_x();
    if (lx) {
      near_dock_ = (*lx - geom_.bumper_x) < up_.until_cm / 100.0;
      if (!((*lx - geom_.bumper_x) < 0.20)) {
        docked_ = false;
      }
    } else if (docked_ && unseen_means_away(r.fail, undock_->scans())) {
      near_dock_ = false;
      docked_ = false;
    }
    fail_locked(r.fail, r.why);
  }

  double resume_remaining_deg_locked()
  {
    const double g = pending_remaining_deg_;
    const auto s = vd::read_dock_spot(dock_yaml_);
    if (!amcl_ || !s || s->map_id != map_id_) {
      return g;
    }
    const double away = vd::wrap_angle(s->fwd_yaw + M_PI);
    const double a = vd::deg(vd::wrap_angle(away - amcl_->yaw)) * static_cast<double>(up_.turn_dir);
    const double diff = vd::deg(vd::wrap_angle(vd::rad(a - g)));
    if (std::abs(diff) > 20.0) {
      RCLCPP_WARN(get_logger(), "남은 회전: 자이로 %+.1f° 와 AMCL %+.1f° 가 다름 — AMCL 값을 쓴다", g, a);
      return a;
    }
    return g;
  }

  void fail_locked(vd::Fail f, const std::string & why)
  {
    last_result_ = "failed";
    last_reason_ = vd::fail_name(f);
    last_why_ = why;
    RCLCPP_WARN(get_logger(), "%s 실패: %s (%s)", last_action_.c_str(), last_reason_.c_str(), why.c_str());
    end_job_locked();
  }

  bool refine_spot_locked(const vd::DockPose & measured, bool visited)
  {
    const auto s = vd::read_dock_spot(dock_yaml_);
    if (!s || s->map_id != map_id_ || s->visited_ok || !amcl_) {
      return false;
    }
    vd::DockSpot n = vd::spot_from_pose(*amcl_, measured);
    n.map_id = s->map_id;
    n.stage_cm = s->stage_cm;
    n.target_cm = s->target_cm;
    n.saved_at = now_text();
    n.source = "v_detect";
    n.visited_ok = visited;
    n.refined = true;
    n.refined_from = s->refined ? s->refined_from : s->source;
    n.refined_cm = std::hypot(n.apex_x - s->apex_x, n.apex_y - s->apex_y) * 100.0 +
      (s->refined ? s->refined_cm : 0.0);
    n.refined_deg = std::abs(vd::deg(vd::wrap_angle(n.fwd_yaw - s->fwd_yaw))) +
      (s->refined ? s->refined_deg : 0.0);
    std::string err;
    if (!vd::write_dock_spot(dock_yaml_, n, &err)) {
      RCLCPP_WARN(get_logger(), "도크 위치 고쳐 저장 실패: %s", err.c_str());
      return false;
    }
    last_refined_ = true;
    last_refined_cm_ = n.refined_cm;
    last_refined_deg_ = n.refined_deg;
    RCLCPP_INFO(
      get_logger(), "도크 위치를 라이다로 잰 값으로 고쳐 저장(%s 에서): %.1f cm · %.1f°%s",
      n.refined_from.c_str(), n.refined_cm, n.refined_deg, visited ? " · 가 본 자리" : "");
    return true;
  }

  void mark_visited_locked()
  {
    auto s = vd::read_dock_spot(dock_yaml_);
    if (s && s->map_id == map_id_ && !s->visited_ok) {
      s->visited_ok = true;
      std::string err;
      if (!vd::write_dock_spot(dock_yaml_, *s, &err)) {
        RCLCPP_WARN(get_logger(), "dock.yaml visited_ok 기록 실패: %s", err.c_str());
      }
    }
  }

  void boot_tick_locked(double now)
  {
    if (find_streak_ >= 5 && find_det_) {
      const auto p = find_pose_;
      const double bumper_apex = p.x - geom_.bumper_x;
      if (bumper_apex <= ap_.target_cm / 100.0 + 0.05 && std::abs(p.y) < 0.05 &&
        std::abs(p.th) < vd::rad(5.0))
      {
        docked_ = true;
        near_dock_ = true;
        RCLCPP_INFO(
          get_logger(), "켜질 때 도크 안: 범퍼→꼭짓점 %.1f cm 좌우 %+.1f cm", bumper_apex * 100.0,
          p.y * 100.0);
        begin_pose_init_locked(PoseInit::kForce, p);
      } else {
        near_dock_ = bumper_apex < up_.until_cm / 100.0;
      }
      job_ = Job::kNone;
      release_subs_locked();
      publish_state_locked();
      return;
    }
    if (now - job_t0_ > boot_check_s_) {
      job_ = Job::kNone;
      release_subs_locked();
      publish_state_locked();
    }
  }

  void register_tick_locked(double now)
  {
    if (find_streak_ >= 5 && find_det_ && amcl_) {
      vd::DockSpot s = vd::spot_from_detection(*amcl_, *find_det_);
      s.map_id = map_id_;
      s.stage_cm = ap_.stage_cm;
      s.target_cm = ap_.target_cm;
      s.saved_at = now_text();
      s.source = "v_detect";
      s.visited_ok = false;
      const auto st = vd::staging_pose(s, geom_.bumper_x);
      std::string err;
      if (!vd::write_dock_spot(dock_yaml_, s, &err)) {
        register_ok_ = false;
        register_reply_ = "{\"reason\":\"write_failed\",\"why\":" + jstr(err) + "}";
      } else {
        const double moved = std::hypot(st.x - amcl_->x, st.y - amcl_->y);
        const double turned = vd::deg(vd::wrap_angle(st.yaw - amcl_->yaw));
        std::ostringstream o;
        o << "{\"staging\":{\"x\":" << fmt("%.4f", st.x) << ",\"y\":" << fmt("%.4f", st.y)
          << ",\"yaw\":" << fmt("%.5f", st.yaw) << "},\"moved_cm\":" << fmt("%.1f", moved * 100.0)
          << ",\"turned_deg\":" << fmt("%.1f", turned) << ",\"bumper_to_apex_cm\":"
          << fmt("%.1f", (find_pose_.x - geom_.bumper_x) * 100.0) << "}";
        register_ok_ = true;
        register_reply_ = o.str();
        RCLCPP_INFO(get_logger(), "도크 위치 저장: %s", register_reply_.c_str());
      }
      end_job_locked();
      return;
    }
    if (now - job_t0_ > 3.5) {
      register_ok_ = false;
      if (!amcl_) {
        register_reply_ = R"({"reason":"no_amcl"})";
      } else if (reg_seen_ > 0 && reg_last_) {
        const auto & a = reg_last_->apex;
        const auto pz = vd::pose_in_dock(*reg_last_);
        std::ostringstream o;
        o << "{\"reason\":\"unstable\",\"dist_m\":" << fmt("%.2f", std::hypot(a.x, a.y))
          << ",\"bearing_deg\":" << fmt("%.1f", vd::deg(std::atan2(a.y, a.x)))
          << ",\"oblique_deg\":" << fmt("%.1f", std::abs(vd::deg(std::atan2(pz.y, pz.x)))) << "}";
        register_reply_ = o.str();
      } else {
        register_reply_ = R"({"reason":"not_found"})";
      }
      end_job_locked();
    }
  }

  enum class PoseInit { kNone, kForce, kCheck };

  void ensure_amcl_sub_locked()
  {
    if (!amcl_sub_) {
      auto opt = rclcpp::SubscriptionOptions();
      opt.callback_group = main_group_;
      amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", rclcpp::QoS(1).transient_local().reliable(),
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {on_amcl(m);}, opt);
    }
  }

  void begin_pose_init_locked(PoseInit mode, const vd::DockPose & measured)
  {
    const auto s = vd::read_dock_spot(dock_yaml_);
    if (!s || s->map_id != map_id_) {
      RCLCPP_WARN(get_logger(), "도크 위치가 저장되지 않아 AMCL 초기 위치를 잡지 않음(%s)", dock_yaml_.c_str());
      return;
    }
    if (s->source == "map_pick" && !s->visited_ok) {
      RCLCPP_INFO(get_logger(), "도크 위치가 지도에서 지정한 값이라(미확인) AMCL 초기 위치를 잡지 않음");
      return;
    }
    pose_target_ = vd::spot_to_map(*s, measured);
    pose_init_ = mode;
    pose_t0_ = mono_now();
    pose_sent_t_ = -1.0;
    pose_far_ = 0;
    amcl_.reset();
    ensure_amcl_sub_locked();
    if (!pose_timer_) {
      pose_timer_ = create_wall_timer(1s, [this]() {on_pose_timer();}, main_group_);
    }
  }

  void stop_pose_init_locked(const char * why)
  {
    if (pose_init_ == PoseInit::kNone) {
      return;
    }
    pose_init_ = PoseInit::kNone;
    pose_result_ = why;
    if (pose_timer_) {
      pose_timer_->cancel();
      pose_timer_.reset();
    }
    RCLCPP_INFO(get_logger(), "AMCL 초기 위치: %s(그만 보냄)", why);
  }

  void on_pose_timer()
  {
    std::lock_guard<std::mutex> lk(m_);
    const double now = mono_now();
    const auto close_to_target = [this]() {
        return amcl_ && std::hypot(amcl_->x - pose_target_.x, amcl_->y - pose_target_.y) <= 0.10 &&
               std::abs(vd::wrap_angle(amcl_->yaw - pose_target_.yaw)) <= vd::rad(3.0);
      };
    bool done = false;
    if (pose_init_ == PoseInit::kCheck) {
      if (pose_sent_t_ < 0) {
        if (amcl_ && close_to_target()) {
          done = true;
          pose_result_ = "kept";
        } else if (amcl_ || now - pose_t0_ > 3.0) {
          send_initial_pose_locked(now);
        }
      } else if (amcl_ && amcl_t_ > pose_sent_t_ && close_to_target()) {
        done = true;
        pose_result_ = "set";
      }
    } else if (pose_init_ == PoseInit::kForce) {
      const bool replied = amcl_ && pose_sent_t_ > 0 && amcl_t_ > pose_sent_t_;
      if (replied && close_to_target()) {
        done = true;
        pose_result_ = "set";
      } else if (replied && ++pose_far_ >= 3) {
        done = true;
        pose_result_ = "set_far";
      } else if (init_pub_->get_subscription_count() > 0) {
        send_initial_pose_locked(now);
      }
    }
    const double limit = pose_init_ == PoseInit::kForce ? 90.0 : 15.0;
    if (!done && now - pose_t0_ > limit) {
      done = true;
      pose_result_ = "timeout";
    }
    if (done || pose_init_ == PoseInit::kNone) {
      RCLCPP_INFO(get_logger(), "AMCL 초기 위치: %s", pose_result_.c_str());
      pose_init_ = PoseInit::kNone;
      if (pose_timer_) {
        pose_timer_->cancel();
        pose_timer_.reset();
      }
      if (job_ != Job::kRegister) {
        amcl_sub_.reset();
      }
      publish_state_locked();
    }
  }

  void send_initial_pose_locked(double now)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped m;
    m.header.frame_id = "map";
    m.header.stamp = this->now();
    m.pose.pose.position.x = pose_target_.x;
    m.pose.pose.position.y = pose_target_.y;
    m.pose.pose.orientation.z = std::sin(pose_target_.yaw / 2.0);
    m.pose.pose.orientation.w = std::cos(pose_target_.yaw / 2.0);
    m.pose.covariance[0] = 0.05 * 0.05;
    m.pose.covariance[7] = 0.05 * 0.05;
    m.pose.covariance[35] = std::pow(vd::rad(3.0), 2);
    init_pub_->publish(m);
    pose_sent_t_ = now;
  }

  void publish_state_locked()
  {
    std::string state = "idle";
    if (job_ == Job::kApproach || job_ == Job::kRetreat) {
      state = "docking";
    } else if (job_ == Job::kUndock) {
      state = "undocking";
    } else if (docked_) {
      state = "docked";
    }
    std::ostringstream o;
    o << "{\"state\":" << jstr(state) << ",\"job\":" << jstr(job_name(job_))
      << ",\"phase\":" << jstr(job_ == Job::kNone ? "" : phase_)
      << ",\"holding\":" << (holding_ && job_ != Job::kNone ? "true" : "false")
      << ",\"near_dock\":" << (near_dock_ ? "true" : "false")
      << ",\"undock_pending\":" << (undock_pending_ ? "true" : "false")
      << ",\"attempt\":" << attempt_
      << ",\"last\":{\"action\":" << jstr(last_action_) << ",\"result\":" << jstr(last_result_)
      << ",\"reason\":" << jstr(last_reason_) << ",\"why\":" << jstr(last_why_)
      << ",\"err_x_cm\":" << fmt("%.2f", err_x_) << ",\"err_y_cm\":" << fmt("%.2f", err_y_)
      << ",\"err_th_deg\":" << fmt("%.2f", err_th_)
      << ",\"refined\":" << (last_refined_ ? "true" : "false")
      << ",\"refined_cm\":" << fmt("%.1f", last_refined_cm_)
      << ",\"refined_deg\":" << fmt("%.1f", last_refined_deg_) << "}"
      << ",\"pose_init\":" << jstr(pose_init_ == PoseInit::kNone ? pose_result_ : "pending")
      << ",\"seq\":" << ++seq_ << "}";
    std_msgs::msg::String m;
    m.data = o.str();
    state_pub_->publish(m);
  }

  void open_trace_locked()
  {
    if (trace_dir_.empty() || !motion_job(job_)) {
      return;
    }
    std::error_code ec;
    std::filesystem::create_directories(trace_dir_, ec);
    char name[64];
    std::time_t t = std::time(nullptr);
    std::strftime(name, sizeof(name), "%Y%m%d_%H%M%S", std::localtime(&t));
    if (trace_.is_open()) {
      trace_.close();
    }
    trace_.clear();
    trace_.open(trace_dir_ + "/" + job_name(job_) + "_" + name + ".csv");
    trace_ << "t_s,job,phase,holding\n";
  }

  void trace_locked(double now)
  {
    if (!trace_.is_open()) {
      return;
    }
    trace_ << fmt("%.3f", now - job_t0_) << "," << job_name(job_) << "," << phase_ << ","
           << (holding_ ? 1 : 0) << "\n";
  }

  void close_trace_locked()
  {
    if (trace_.is_open()) {
      trace_ << "# result," << last_action_ << "," << last_result_ << "," << last_reason_ << "\n";
      trace_.close();
    }
  }

  std::mutex m_;
  vd::RobotGeometry geom_;
  vd::ApproachParams ap_;
  vd::UndockParams up_;
  std::string dock_yaml_;
  std::string map_id_;
  bool auto_initial_pose_{true};
  double boot_check_s_{20.0};
  std::string trace_dir_;
  std::ofstream trace_;

  rclcpp::CallbackGroup::SharedPtr srv_group_;
  rclcpp::CallbackGroup::SharedPtr main_group_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_watch_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> us_subs_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::TimerBase::SharedPtr job_timer_;
  rclcpp::TimerBase::SharedPtr pose_timer_;

  Job job_{Job::kNone};
  double job_t0_{0.0};
  int tail_ticks_{0};
  std::unique_ptr<vd::ApproachController> approach_;
  std::unique_ptr<vd::UndockController> undock_;
  std::unique_ptr<vd::VDetector> finder_;
  int find_streak_{0};
  std::optional<vd::VDetection> find_det_;
  vd::DockPose find_pose_;
  int attempt_{0};
  bool docked_{false};
  bool near_dock_{false};
  int reg_seen_{0};
  std::optional<vd::VDetection> reg_last_;
  bool last_refined_{false};
  double last_refined_cm_{0.0};
  double last_refined_deg_{0.0};
  rclcpp::Service<vica_interfaces::srv::SaveHome>::SharedPtr set_srv_;
  bool undock_pending_{false};
  bool pending_turn_only_{false};
  double pending_remaining_deg_{0.0};
  bool resume_turn_wait_{false};
  bool holding_{false};
  std::string phase_;
  std::string last_action_;
  std::string last_result_;
  std::string last_reason_;
  std::string last_why_;
  double err_x_{0.0};
  double err_y_{0.0};
  double err_th_{0.0};
  long seq_{0};
  std::string register_reply_;
  bool register_ok_{false};
  std::optional<vd::Pose2> amcl_;
  double amcl_t_{0.0};
  PoseInit pose_init_{PoseInit::kNone};
  vd::Pose2 pose_target_;
  double pose_t0_{0.0};
  double pose_sent_t_{-1.0};
  int pose_far_{0};
  std::string pose_result_;
  bool cancel_req_{false};
  double job_end_t_{-10.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DockingNode>();
  rclcpp::executors::MultiThreadedExecutor ex(rclcpp::ExecutorOptions(), 2);
  ex.add_node(node);
  ex.spin();
  rclcpp::shutdown();
  return 0;
}
