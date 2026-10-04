// IMU 互补滤波节点：陀螺积分传播 + 加计重力方向校正的互补融合姿态。
// 启动段用静止样本标定陀螺零偏；输出 /imu/data_filtered 供 ekf_filter_node 融合。
// 互补滤波经典形式：angle = α·(angle + gyro·dt) + (1-α)·accel_angle，
// α 取 0.98 即高频信陀螺(抗加计振动噪声)、低频信加计(压陀螺漂移)。
// yaw 无磁力计可观，仅由零偏校正后的 z 轴陀螺积分，属相对航向。

#include <array>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

using sensor_msgs::msg::Imu;

namespace {

constexpr double kGravity = 9.81;  // xacro IMU 在平地上静止标定，量纲 m/s^2

double wrap_angle(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0) a += 2.0 * M_PI;
  return a - M_PI;
}

void fill_diag_covariance(std::array<double, 9>& cov, double variance) {
  cov.fill(0.0);
  cov[0] = cov[4] = cov[8] = variance;
}

}  // namespace

class ImuFilterNode : public rclcpp::Node {
 public:
  ImuFilterNode() : Node("imu_filter") {
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data");
    const auto filtered_topic = declare_parameter<std::string>("filtered_topic", "/imu/data_filtered");
    alpha_ = declare_parameter<double>("alpha", 0.98);
    calib_samples_ = declare_parameter<int>("calibration_samples", 400);
    // 静止判定：|mean(|a|) - g| / g 超过该阈值视为标定期不在静止，零偏置 0
    stationary_tolerance_ = declare_parameter<double>("stationary_tolerance", 0.15);
    // 加计重力校正的信任门槛：|norm - g| / g 超过该值(机动/振动)则该帧只做陀螺传播
    accel_trust_tolerance_ = declare_parameter<double>("accel_trust_tolerance", 0.3);
    orientation_variance_ = declare_parameter<double>("orientation_variance", 0.01);
    angular_velocity_variance_ = declare_parameter<double>("angular_velocity_variance", 1.0e-4);
    linear_acceleration_variance_ = declare_parameter<double>("linear_acceleration_variance", 0.09);

    // 订阅端 SensorDataQoS：兼容 Gazebo 的 reliable 发布与真实驱动可能的 best_effort
    imu_sub_ = create_subscription<Imu>(
        imu_topic, rclcpp::SensorDataQoS(),
        [this](Imu::ConstSharedPtr msg) { on_imu(*msg); });
    // 发布端 reliable：robot_localization 订阅端默认 reliable，best_effort 会匹配失败
    filtered_pub_ = create_publisher<Imu>(filtered_topic, rclcpp::QoS(10));

    RCLCPP_INFO(get_logger(), "'%s' -> '%s' | alpha=%.2f calib=%d",
                imu_topic.c_str(), filtered_topic.c_str(), alpha_, calib_samples_);
  }

 private:
  void on_imu(const Imu& msg) {
    const double ax = msg.linear_acceleration.x;
    const double ay = msg.linear_acceleration.y;
    const double az = msg.linear_acceleration.z;

    // ---------- 陀螺零偏标定：前 N 帧（要求静止）取均值 ----------
    if (calib_count_ < calib_samples_) {
      bias_sum_x_ += msg.angular_velocity.x;
      bias_sum_y_ += msg.angular_velocity.y;
      bias_sum_z_ += msg.angular_velocity.z;
      accel_norm_sum_ += std::sqrt(ax * ax + ay * ay + az * az);
      ++calib_count_;
      bias_x_ = bias_sum_x_ / calib_count_;  // 标定期用运行均值即时校正
      bias_y_ = bias_sum_y_ / calib_count_;
      bias_z_ = bias_sum_z_ / calib_count_;
      if (calib_count_ == calib_samples_) finish_calibration();
    }

    // ---------- dt：用消息时间戳（仿真时钟一致），异常帧跳过积分 ----------
    double dt = 0.0;
    if (have_stamp_) {
      const rclcpp::Duration d = rclcpp::Time(msg.header.stamp) - last_stamp_;
      dt = d.seconds();
    }
    last_stamp_ = msg.header.stamp;
    have_stamp_ = true;
    const bool dt_valid = dt > 0.0 && dt < 0.5;

    const double gx = msg.angular_velocity.x - bias_x_;
    const double gy = msg.angular_velocity.y - bias_y_;
    const double gz = msg.angular_velocity.z - bias_z_;

    // ---------- 陀螺传播 ----------
    if (dt_valid) {
      roll_ = wrap_angle(roll_ + gx * dt);
      pitch_ = wrap_angle(pitch_ + gy * dt);
      yaw_ = wrap_angle(yaw_ + gz * dt);
    }

    // ---------- 加计校正：仅当合加速度接近 g（重力主导）时信任 ----------
    const double norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (norm > 1e-6 && std::abs(norm - kGravity) / kGravity <= accel_trust_tolerance_) {
      const double roll_acc = std::atan2(ay, az);
      const double pitch_acc = std::atan2(-ax, std::sqrt(ay * ay + az * az));
      roll_ = wrap_angle(alpha_ * roll_ + (1.0 - alpha_) * roll_acc);
      pitch_ = wrap_angle(alpha_ * pitch_ + (1.0 - alpha_) * pitch_acc);
    }

    // ---------- 发布滤波结果：姿态来自滤波器，陀螺为零偏校正值，加计透传 ----------
    Imu out;
    out.header = msg.header;
    tf2::Quaternion q;
    q.setRPY(roll_, pitch_, yaw_);
    out.orientation.x = q.x();
    out.orientation.y = q.y();
    out.orientation.z = q.z();
    out.orientation.w = q.w();
    fill_diag_covariance(out.orientation_covariance, orientation_variance_);
    out.angular_velocity = msg.angular_velocity;
    out.angular_velocity.x = gx;
    out.angular_velocity.y = gy;
    out.angular_velocity.z = gz;
    fill_diag_covariance(out.angular_velocity_covariance, angular_velocity_variance_);
    out.linear_acceleration = msg.linear_acceleration;
    fill_diag_covariance(out.linear_acceleration_covariance, linear_acceleration_variance_);
    filtered_pub_->publish(out);
  }

  void finish_calibration() {
    const double mean_norm = accel_norm_sum_ / calib_count_;
    if (std::abs(mean_norm - kGravity) / kGravity > stationary_tolerance_) {
      bias_x_ = bias_y_ = bias_z_ = 0.0;
      RCLCPP_WARN(get_logger(),
                  "标定期 |a|=%.3f 非 g（可能未静止），陀螺零偏置 0，姿态纯陀螺积分",
                  mean_norm);
      return;
    }
    RCLCPP_INFO(get_logger(), "陀螺零偏标定完成: [%.6f %.6f %.6f] rad/s (%d 帧)",
                bias_x_, bias_y_, bias_z_, calib_count_);
  }

  // 参数
  double alpha_{0.98};
  int calib_samples_{400};
  double stationary_tolerance_{0.15};
  double accel_trust_tolerance_{0.3};
  double orientation_variance_{0.01};
  double angular_velocity_variance_{1.0e-4};
  double linear_acceleration_variance_{0.09};

  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<Imu>::SharedPtr filtered_pub_;

  // 滤波状态
  double roll_{0.0}, pitch_{0.0}, yaw_{0.0};
  double bias_x_{0.0}, bias_y_{0.0}, bias_z_{0.0};
  double bias_sum_x_{0.0}, bias_sum_y_{0.0}, bias_sum_z_{0.0};
  double accel_norm_sum_{0.0};
  int calib_count_{0};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  bool have_stamp_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuFilterNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
