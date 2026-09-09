// IMU 를 base_link 좌표계로 옮겨 다시 내보낸다 (파이썬 판의 C++ 이식).
//
// 왜 C++ 인가 (2026-09-09 실측):
//
//     D455 자이로는 200 Hz 가 하드웨어 최저라 입력을 줄일 수 없다. 그런데
//     파이썬은 그 194 Hz 를 **받기만 해도 16.4 %** 를 쓴다(같은 조건 C++ 는
//     0.0 %). 어댑터 전체는 25.2 % 였고, 그 탓에 EKF 가 설정 30 Hz 를 못 지켜
//     9.6 Hz 로 절뚝였다. 그 여유 부족이 9/7 TF 공백 사고를 키웠다.
//
// 옮기면서 계약은 한 줄도 바꾸지 않았다. 파라미터 이름·기본값·로그 문구가
// 파이썬 판과 같아서 운영 중에 구분 없이 바꿔 끼울 수 있다. 편향 로직은
// gyro_bias 로 분리해 시험 19 개를 먼저 통과시켰다.
//
// 되돌리려면 터미네이터 칸의 실행 명령만 파이썬 판으로 되돌리면 된다:
//     ros2 run vica_sensor_adapters imu_base_link_adapter
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "vica_sensor_adapters_cpp/gyro_bias.hpp"

namespace vica_sensor_adapters_cpp
{

using Matrix3 = std::array<std::array<double, 3>, 3>;

/// 쿼터니언을 회전 행렬로. 크기가 0 이면 회전을 지어내지 않고 실패한다.
bool quat_to_matrix(const geometry_msgs::msg::Quaternion & q, Matrix3 & m)
{
  double x = q.x, y = q.y, z = q.z, w = q.w;
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (norm == 0.0) {
    return false;
  }
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;

  m[0] = {1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)};
  m[1] = {2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)};
  m[2] = {2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)};
  return true;
}

void rotate_vector(const Matrix3 & m, geometry_msgs::msg::Vector3 & v)
{
  const std::array<double, 3> in{v.x, v.y, v.z};
  std::array<double, 3> out{0.0, 0.0, 0.0};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      out[row] += m[row][col] * in[col];
    }
  }
  v.x = out[0];
  v.y = out[1];
  v.z = out[2];
}

/// R * C * R^T. 첫 원소가 음수면 "무효" 표시라 그대로 둔다(IMU 규약).
void rotate_covariance(const Matrix3 & m, std::array<double, 9> & cov)
{
  if (cov[0] < 0.0) {
    return;
  }
  std::array<double, 9> out{};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      double value = 0.0;
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          value += m[row][i] * cov[i * 3 + j] * m[col][j];
        }
      }
      out[row * 3 + col] = value;
    }
  }
  cov = out;
}

class ImuBaseLinkAdapter : public rclcpp::Node
{
public:
  ImuBaseLinkAdapter()
  : Node("imu_base_link_adapter")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/camera/camera/imu");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/imu/base_link");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.05);

    // 정지 중 자이로 편향 보정. 0 이면 끈다. 근거는 파이썬 판 주석 참고
    // (2026-08-01 실측: 표본 없음 161 deg/hour -> 4000 표본 4.7 deg/hour).
    const int sample_count = declare_parameter<int>("gyro_bias_sample_count", 1000);
    const double max_rate = declare_parameter<double>("gyro_bias_max_rate", 0.05);
    // ZUPT: 정차할 때마다 다시 잰다(2026-08-30). max_dev 가 가장 중요한 값으로,
    // 직진(마스트 진동)을 정차로 착각하지 않게 막는다.
    const int refresh_count = declare_parameter<int>("gyro_bias_refresh_count", 120);
    const double refresh_alpha = declare_parameter<double>("gyro_bias_refresh_alpha", 0.2);
    const double max_dev = declare_parameter<double>("gyro_bias_max_dev", 0.02);
    const double max_jump = declare_parameter<double>("gyro_bias_max_jump", 0.01);

    // 출력 주파수 상한. 0 이하면 제한하지 않는다(입력 그대로).
    // 40 Hz 는 EKF 사용 주기(30 Hz)의 1.33 배다. 200 Hz 표본을 버리지 않고
    // 발행 주기마다 평균 내면 5 표본 평균이 되어 백색잡음이 sqrt(5) 배 준다.
    const double rate = declare_parameter<double>("publish_rate_hz", 40.0);
    min_period_sec_ = (rate > 0.0) ? (1.0 / rate) : 0.0;

    bias_ = std::make_unique<GyroBiasEstimator>(
      sample_count, max_rate, refresh_count, refresh_alpha, max_dev, max_jump);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS qos(rclcpp::KeepLast(50));
    qos.best_effort();

    pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic, 10);
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic, qos,
      [this](sensor_msgs::msg::Imu::SharedPtr msg) {imu_callback(*msg);});

    RCLCPP_INFO(
      get_logger(), "%s -> %s in %s",
      input_topic.c_str(), output_topic.c_str(), target_frame_.c_str());
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu & msg)
  {
    // 모든 표본을 합산에 넣는다(평균 내기). 스로틀에 걸려도 버리지 않는다.
    acc_gyro_[0] += msg.angular_velocity.x;
    acc_gyro_[1] += msg.angular_velocity.y;
    acc_gyro_[2] += msg.angular_velocity.z;
    acc_lin_[0] += msg.linear_acceleration.x;
    acc_lin_[1] += msg.linear_acceleration.y;
    acc_lin_[2] += msg.linear_acceleration.z;
    ++acc_n_;

    // 스로틀은 TF·회전보다 **먼저** 한다. 뒤에 두면 CPU 를 아끼지 못한다.
    if (min_period_sec_ > 0.0) {
      const double now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      if (now - last_publish_ < min_period_sec_) {
        return;
      }
      last_publish_ = now;
    }

    // 평균을 꺼내고 합산을 비운다. TF 실패로 이번 발행을 건너뛰더라도 비운다 —
    // 기동 초기 TF 대기 동안 쌓인 낡은 표본이 첫 발행에 한꺼번에 평균되는 것을
    // 막는다.
    const double n = static_cast<double>(acc_n_);
    const std::array<double, 3> avg_gyro{acc_gyro_[0] / n, acc_gyro_[1] / n, acc_gyro_[2] / n};
    const std::array<double, 3> avg_lin{acc_lin_[0] / n, acc_lin_[1] / n, acc_lin_[2] / n};
    acc_n_ = 0;
    acc_gyro_ = {0.0, 0.0, 0.0};
    acc_lin_ = {0.0, 0.0, 0.0};

    Matrix3 matrix;
    if (!matrix_for(msg.header.frame_id, matrix)) {
      return;
    }

    sensor_msgs::msg::Imu out;
    out.header = msg.header;
    out.header.frame_id = target_frame_;

    out.orientation = msg.orientation;
    out.orientation_covariance = msg.orientation_covariance;

    out.angular_velocity.x = avg_gyro[0];
    out.angular_velocity.y = avg_gyro[1];
    out.angular_velocity.z = avg_gyro[2];
    rotate_vector(matrix, out.angular_velocity);

    // 편향은 회전 뒤 base_link 기준으로 다룬다. EKF 가 쓰는 축이 그것이다.
    // 선가속도에는 적용하지 않는다 — 중력은 실제 값이다.
    apply_gyro_bias(out.angular_velocity);

    out.angular_velocity_covariance = msg.angular_velocity_covariance;
    rotate_covariance(matrix, out.angular_velocity_covariance);

    out.linear_acceleration.x = avg_lin[0];
    out.linear_acceleration.y = avg_lin[1];
    out.linear_acceleration.z = avg_lin[2];
    rotate_vector(matrix, out.linear_acceleration);
    out.linear_acceleration_covariance = msg.linear_acceleration_covariance;
    rotate_covariance(matrix, out.linear_acceleration_covariance);

    pub_->publish(out);
  }

  /// 캐시된 회전 행렬을 준다. frame_id 가 바뀌면 다시 조회한다.
  ///
  /// base_link <- IMU frame 은 URDF 고정 변환이라 한 번만 조회하면 된다.
  /// lookup_transform 이 파이썬 판 비용의 큰 몫이었다(주행 실측 25~33 %).
  bool matrix_for(const std::string & frame_id, Matrix3 & out)
  {
    if (has_tf_ && frame_id == tf_frame_id_) {
      out = tf_matrix_;
      return true;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        target_frame_, frame_id, tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for TF %s <- %s: %s",
        target_frame_.c_str(), frame_id.c_str(), ex.what());
      return false;
    }

    Matrix3 m;
    if (!quat_to_matrix(transform.transform.rotation, m)) {
      RCLCPP_WARN(get_logger(), "Received invalid TF rotation quaternion");
      return false;
    }

    tf_matrix_ = m;
    tf_frame_id_ = frame_id;
    has_tf_ = true;
    out = m;
    return true;
  }

  /// 정지 중 추정한 편향을 뺀다. 확정 전이면 원값이 그대로 나간다.
  void apply_gyro_bias(geometry_msgs::msg::Vector3 & gyro)
  {
    bias_->add(gyro.x, gyro.y, gyro.z);
    const Vec3 corrected = bias_->correct(gyro.x, gyro.y, gyro.z);
    gyro.x = corrected[0];
    gyro.y = corrected[1];
    gyro.z = corrected[2];

    // ZUPT 갱신은 조용히 일어나므로 로그로 확인할 길을 둔다. 실주행에서
    // "정차를 잡고 있는가"를 보는 유일한 창이다.
    if (bias_->refresh_count() != bias_refresh_seen_) {
      bias_refresh_seen_ = bias_->refresh_count();
      const double bz = bias_->bias()[2];
      RCLCPP_INFO(
        get_logger(),
        "Gyro bias refreshed (#%d) at stop: yaw bias %+.6f rad/s (%+.1f deg/hour)",
        bias_->refresh_count(), bz, bz * 180.0 / M_PI * 3600.0);
    }

    if (bias_reported_) {
      return;
    }

    if (bias_->ready()) {
      const Vec3 b = bias_->bias();
      const double drift = b[2] * 180.0 / M_PI * 3600.0;
      RCLCPP_INFO(
        get_logger(),
        "Gyro bias calibrated over %d samples: (%+.6f, %+.6f, %+.6f) rad/s. "
        "Removed yaw drift of %+.1f deg/hour.",
        bias_->collected(), b[0], b[1], b[2], drift);
      bias_reported_ = true;
    } else if (bias_->aborted()) {
      RCLCPP_WARN(
        get_logger(),
        "Gyro bias calibration aborted: motion detected during startup. "
        "Publishing uncorrected rates - yaw will drift. "
        "Restart this node while the robot is stationary.");
      bias_reported_ = true;
    }
  }

  std::string target_frame_;
  double transform_timeout_sec_{0.05};
  double min_period_sec_{0.0};
  double last_publish_{0.0};

  int acc_n_{0};
  std::array<double, 3> acc_gyro_{0.0, 0.0, 0.0};
  std::array<double, 3> acc_lin_{0.0, 0.0, 0.0};

  Matrix3 tf_matrix_{};
  std::string tf_frame_id_;
  bool has_tf_{false};

  std::unique_ptr<GyroBiasEstimator> bias_;
  bool bias_reported_{false};
  int bias_refresh_seen_{0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

}  // namespace vica_sensor_adapters_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vica_sensor_adapters_cpp::ImuBaseLinkAdapter>());
  rclcpp::shutdown();
  return 0;
}
