// 转换节点：信封 JSON <-> ROS 原生消息。
// 自带信封编解码（vendored nlohmann 单头），仅依赖协议规格，对上游项目零感知。
// 规格见 docs/2026-10-03-design-ros2-robot-side.md §3/§5/§7/§8。

#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using std_msgs::msg::String;
using namespace std::chrono_literals;

namespace {

using Json = nlohmann::ordered_json;  // 保持文档字段顺序，便于抓包对照

constexpr char kTypeCmdVel[] = "cmd_vel";
constexpr char kTypeTelemetry[] = "telemetry";
constexpr char kTypeHeartbeat[] = "heartbeat";

int64_t now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

double finite_or_zero(double v) { return std::isfinite(v) ? v : 0.0; }

struct Envelope {
  Json body;
  std::string type;
  int64_t seq = 0;
  int64_t ts_ms = 0;
};

// §3.5：JSON 语法错 / 缺 type、seq、ts_ms / 字段类型不符 -> false。
// 数值字段按 double 解读，整数形式也接受。
bool parse_envelope(const std::string& line, Envelope& env) {
  env.body = Json::parse(line, nullptr, /*allow_exceptions=*/false);
  if (env.body.is_discarded() || !env.body.is_object()) return false;
  const auto type_it = env.body.find("type");
  if (type_it == env.body.end() || !type_it->is_string()) return false;
  const auto seq_it = env.body.find("seq");
  if (seq_it == env.body.end() || !seq_it->is_number()) return false;
  const auto ts_it = env.body.find("ts_ms");
  if (ts_it == env.body.end() || !ts_it->is_number()) return false;
  env.type = type_it->get<std::string>();
  env.seq = seq_it->get<int64_t>();
  env.ts_ms = ts_it->get<int64_t>();
  return true;
}

bool read_vec3(const Json& parent, const char* key, double out[3]) {
  const auto it = parent.find(key);
  if (it == parent.end() || !it->is_object()) return false;
  const char* axes[3] = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    const auto field = it->find(axes[i]);
    if (field == it->end() || !field->is_number()) return false;
    out[i] = field->get<double>();
    if (!std::isfinite(out[i])) return false;  // JSON 无法表达的 inf/nan 整包作废
  }
  return true;
}

// §3.2：linear/angular 各 x y z 六个 double 全部必填、无默认值；缺任一 -> false（整包丢弃）
bool extract_twist(const Json& body, Twist& twist) {
  const auto data = body.find("data");
  if (data == body.end() || !data->is_object()) return false;
  double linear[3];
  double angular[3];
  if (!read_vec3(*data, "linear", linear) || !read_vec3(*data, "angular", angular)) {
    return false;
  }
  twist.linear.x = linear[0];
  twist.linear.y = linear[1];
  twist.linear.z = linear[2];
  twist.angular.x = angular[0];
  twist.angular.y = angular[1];
  twist.angular.z = angular[2];
  return true;
}

}  // namespace

class TranslatorNode : public rclcpp::Node {
 public:
  TranslatorNode() : Node("translator") {
    const auto uplink_topic = declare_parameter<std::string>("uplink_topic", "/wheels/uplink");
    const auto downlink_topic =
        declare_parameter<std::string>("downlink_topic", "/wheels/downlink");
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    telemetry_rate_hz_ = declare_parameter<double>("telemetry_rate_hz", 10.0);
    cmd_vel_timeout_ms_ = declare_parameter<int64_t>("cmd_vel_timeout_ms", 500);
    heartbeat_rate_hz_ = declare_parameter<double>("heartbeat_rate_hz", 1.0);

    // §6 QoS：/cmd_vel reliable depth 1；downlink best_effort depth 10；
    //         uplink reliable depth 10；/odom best_effort depth 10（REP 2003）
    cmd_pub_ = create_publisher<Twist>(cmd_vel_topic, rclcpp::QoS(1));
    downlink_pub_ = create_publisher<String>(downlink_topic, rclcpp::QoS(10).best_effort());
    uplink_sub_ = create_subscription<String>(
        uplink_topic, rclcpp::QoS(10),
        [this](String::ConstSharedPtr msg) { on_uplink(msg->data); });
    odom_sub_ = create_subscription<Odometry>(
        odom_topic, rclcpp::QoS(10).best_effort(),
        [this](Odometry::ConstSharedPtr msg) { on_odom(msg); });

    last_cmd_time_ = std::chrono::steady_clock::now();
    watchdog_timer_ = create_wall_timer(100ms, [this] { watchdog_tick(); });
    if (telemetry_rate_hz_ > 0.0) {
      // 0 = 每条 odom 都转（on_odom 直转，不起定时器）；周期下限 1ms 防误配忙等
      const auto period = std::max(1ms, std::chrono::duration_cast<std::chrono::milliseconds>(
                                           std::chrono::duration<double>(1.0 / telemetry_rate_hz_)));
      telemetry_timer_ = create_wall_timer(period, [this] { telemetry_tick(); });
    }
    if (heartbeat_rate_hz_ > 0.0) {
      const auto period = std::max(1ms, std::chrono::duration_cast<std::chrono::milliseconds>(
                                           std::chrono::duration<double>(1.0 / heartbeat_rate_hz_)));
      heartbeat_timer_ = create_wall_timer(period, [this] { publish_heartbeat(); });
    }

    RCLCPP_INFO(get_logger(),
                "uplink='%s' -> %s | %s -> downlink='%s' | telemetry=%.1fHz watchdog=%ldms "
                "heartbeat=%.1fHz",
                uplink_topic.c_str(), cmd_vel_topic.c_str(), odom_topic.c_str(),
                downlink_topic.c_str(), telemetry_rate_hz_,
                static_cast<long>(cmd_vel_timeout_ms_), heartbeat_rate_hz_);
  }

 private:
  // ---------- 上行：信封 -> /cmd_vel（§5） ----------

  void on_uplink(const std::string& line) {
    Envelope env;
    if (!parse_envelope(line, env)) {
      ++parse_failures_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "信封解析失败，整包丢弃（累计 %zu）",
                           parse_failures_);
      return;
    }
    if (env.type == kTypeCmdVel) {
      Twist twist;
      if (!extract_twist(env.body, twist)) {
        ++bad_payloads_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "cmd_vel 缺 data 子字段，整包丢弃（累计 %zu）", bad_payloads_);
        return;
      }
      cmd_pub_->publish(twist);
      last_cmd_time_ = std::chrono::steady_clock::now();
      if (watchdog_active_) {
        watchdog_active_ = false;
        RCLCPP_INFO(get_logger(), "收到新 cmd_vel，watchdog 停止兜底");
      }
    } else if (env.type == kTypeHeartbeat) {
      ++heartbeats_rx_;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "heartbeat x%zu（seq=%ld）",
                           heartbeats_rx_, static_cast<long>(env.seq));
    } else {  // telemetry / 未知类型：丢弃 + 计数（§5）
      ++dropped_envelopes_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "type='%s' 不上行，丢弃（累计 %zu）",
                           env.type.c_str(), dropped_envelopes_);
    }
  }

  // ---------- watchdog：断链兜底停车（§5） ----------

  void watchdog_tick() {
    if (cmd_vel_timeout_ms_ <= 0) return;
    const auto idle = std::chrono::steady_clock::now() - last_cmd_time_;
    if (idle < std::chrono::milliseconds(cmd_vel_timeout_ms_)) return;
    cmd_pub_->publish(Twist{});  // 全零
    if (!watchdog_active_) {
      watchdog_active_ = true;
      RCLCPP_WARN(get_logger(), "%ld ms 无 cmd_vel，watchdog 开始每 100ms 兜底发全零 Twist",
                  static_cast<long>(cmd_vel_timeout_ms_));
    }
  }

  // ---------- 下行：/odom -> telemetry 信封（§5/§7） ----------

  void on_odom(Odometry::ConstSharedPtr msg) {
    if (telemetry_rate_hz_ > 0.0) {
      latest_odom_ = *msg;  // 缓存最新一帧，定时器节流发送
      have_odom_ = true;
      return;
    }
    publish_telemetry(*msg);  // 0 = 每条 odom 都转
  }

  void telemetry_tick() {
    if (!have_odom_) return;
    publish_telemetry(latest_odom_);
  }

  void publish_telemetry(const Odometry& odom) {
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    const auto& q = odom.pose.pose.orientation;
    if (q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0) {
      RCLCPP_WARN_ONCE(get_logger(), "odom 四元数全零（未初始化），欧拉角记 0（§8）");
    } else {
      tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
    }
    const auto& p = odom.pose.pose.position;
    const auto& t = odom.twist.twist;

    Json json;
    json["type"] = kTypeTelemetry;
    json["seq"] = seq_++;
    json["ts_ms"] = now_ms();
    auto& data = json["data"];
    data["position"] = {
        {"x", finite_or_zero(p.x)}, {"y", finite_or_zero(p.y)}, {"z", finite_or_zero(p.z)}};
    data["orientation"] = {{"roll", finite_or_zero(roll)},
                           {"pitch", finite_or_zero(pitch)},
                           {"yaw", finite_or_zero(yaw)}};
    data["linear"] = {{"vx", finite_or_zero(t.linear.x)},
                      {"vy", finite_or_zero(t.linear.y)},
                      {"vz", finite_or_zero(t.linear.z)}};
    data["angular"] = {{"wx", finite_or_zero(t.angular.x)},
                       {"wy", finite_or_zero(t.angular.y)},
                       {"wz", finite_or_zero(t.angular.z)}};

    String msg;
    msg.data = json.dump();
    downlink_pub_->publish(msg);
  }

  void publish_heartbeat() {
    Json json;
    json["type"] = kTypeHeartbeat;
    json["seq"] = seq_++;
    json["ts_ms"] = now_ms();
    String msg;
    msg.data = json.dump();
    downlink_pub_->publish(msg);
  }

  double telemetry_rate_hz_ = 10.0;
  int64_t cmd_vel_timeout_ms_ = 500;
  double heartbeat_rate_hz_ = 1.0;

  rclcpp::Publisher<Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<String>::SharedPtr downlink_pub_;
  rclcpp::Subscription<String>::SharedPtr uplink_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // executor 单线程访问，无锁
  std::chrono::steady_clock::time_point last_cmd_time_;
  bool watchdog_active_ = false;
  int64_t seq_ = 0;
  size_t parse_failures_ = 0;
  size_t bad_payloads_ = 0;
  size_t dropped_envelopes_ = 0;
  size_t heartbeats_rx_ = 0;
  Odometry latest_odom_;
  bool have_odom_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TranslatorNode>());
  rclcpp::shutdown();
  return 0;
}
