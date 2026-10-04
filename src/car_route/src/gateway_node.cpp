// 网关节点：TCP 服务端 + NDJSON 帧定界 + std_msgs/String 透传。
// 只做字节流 <-> 行的搬运与 JSON 合法性校验，不解析任何字段语义。
// 行为规格见 docs/2026-10-03-design-ros2-robot-side.md §4/§8。

#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <utility>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std_msgs::msg::String;
using namespace std::chrono_literals;

namespace {

constexpr size_t kMaxLineBytes = 64 * 1024;     // 行缓冲上限，超限清空重同步
constexpr size_t kMaxQueuedLines = 4096;        // 线程间队列软上限，防失控对端撑爆内存
constexpr size_t kMaxOutBufBytes = 1 << 20;     // 发送缓冲上限，积压超限踢掉慢客户端
constexpr size_t kMaxReadPerCycle = 64 * 1024;  // 单次 POLLIN 读取预算，保证 accept 轮转
constexpr auto kPollTimeout = 200ms;

}  // namespace

class GatewayNode : public rclcpp::Node {
 public:
  GatewayNode() : Node("gateway") {
    port_ = declare_parameter<int>("port", 10000);
    const auto uplink_topic =
        declare_parameter<std::string>("uplink_topic", "/wheels/uplink");
    const auto downlink_topic =
        declare_parameter<std::string>("downlink_topic", "/wheels/downlink");

    // §6：uplink reliable depth 10
    uplink_pub_ = create_publisher<String>(uplink_topic, rclcpp::QoS(10));
    // 下行发布方（translator）为 best_effort，订阅端取一致 QoS 才兼容
    downlink_sub_ = create_subscription<String>(
        downlink_topic, rclcpp::QoS(10).best_effort(),
        [this](String::ConstSharedPtr msg) { enqueue_tx(msg->data); });
    // 上行经线程队列交接，由 executor 侧定时发布（§4 线程模型）
    uplink_drain_ = create_wall_timer(10ms, [this] { drain_uplink(); });

    running_ = true;
    tcp_thread_ = std::thread([this] { tcp_loop(); });

    RCLCPP_INFO(get_logger(), "listening on 0.0.0.0:%d, uplink='%s', downlink='%s'",
                port_, uplink_topic.c_str(), downlink_topic.c_str());
  }

  ~GatewayNode() override {
    running_ = false;
    if (tcp_thread_.joinable()) tcp_thread_.join();
  }

  GatewayNode(const GatewayNode&) = delete;
  GatewayNode& operator=(const GatewayNode&) = delete;

 private:
  // ---------- TCP 线程：只碰 socket 与线程队列，不直接调用 rclcpp ----------

  void tcp_loop() {
    listen_fd_ = make_listener();
    if (listen_fd_ < 0) return;

    int client_fd = -1;
    std::string rx_buf;   // 行缓冲
    std::string out_buf;  // 发送缓冲

    while (running_.load(std::memory_order_relaxed)) {
      // 单客户端：新连接随时顶掉旧连接（半开连接自愈）
      try_accept(listen_fd_, client_fd, rx_buf, out_buf);

      pollfd fds[2];
      int n = 0;
      fds[n++] = {listen_fd_, POLLIN, 0};
      int client_slot = -1;
      if (client_fd >= 0) {
        client_slot = n;
        fds[n++] = {client_fd,
                    static_cast<short>(POLLIN | (out_buf.empty() ? 0 : POLLOUT)), 0};
      }

      const int pr = ::poll(fds, static_cast<nfds_t>(n),
                            static_cast<int>(kPollTimeout.count()));
      if (pr < 0) {
        if (errno == EINTR) continue;
        RCLCPP_ERROR(get_logger(), "poll 失败: %s, TCP 线程退出", std::strerror(errno));
        break;
      }

      // 下行：队列 -> 发送缓冲（无客户端期间丢弃并计数）
      drain_tx(client_fd, out_buf);

      if (client_fd >= 0 && (fds[client_slot].revents & POLLOUT) != 0) {
        flush_out(client_fd, out_buf);
      }
      if (client_fd >= 0 && (fds[client_slot].revents & (POLLIN | POLLERR | POLLHUP)) != 0) {
        if (!read_client(client_fd, rx_buf)) {
          close_client(client_fd, rx_buf, out_buf, "对端断开");
        }
      }
      split_lines(rx_buf);

      if (close_requested_) {  // flush_out / drain_tx 检测到写失败或积压超限
        close_client(client_fd, rx_buf, out_buf, close_reason_);
        close_requested_ = false;
      }
    }

    if (client_fd >= 0) ::close(client_fd);
    ::close(listen_fd_);
    listen_fd_ = -1;
  }

  int make_listener() {
    const int fd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (fd < 0) {
      RCLCPP_ERROR(get_logger(), "socket 创建失败: %s", std::strerror(errno));
      return -1;
    }
    int one = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port_));
    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0 ||
        ::listen(fd, 1) < 0) {
      RCLCPP_ERROR(get_logger(), "监听 0.0.0.0:%d 失败: %s", port_, std::strerror(errno));
      ::close(fd);
      return -1;
    }
    return fd;
  }

  void try_accept(int listen_fd, int& client_fd, std::string& rx_buf, std::string& out_buf) {
    for (;;) {
      const int fd = ::accept4(listen_fd, nullptr, nullptr, SOCK_NONBLOCK);
      if (fd < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return;
        RCLCPP_ERROR(get_logger(), "accept 失败: %s", std::strerror(errno));
        return;
      }
      const bool kicked = client_fd >= 0;
      if (kicked) {
        ::close(client_fd);
        ++kicked_clients_;
      }
      client_fd = fd;
      int one = 1;
      ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
      rx_buf.clear();
      out_buf.clear();
      RCLCPP_INFO(get_logger(), "客户端接入 fd=%d%s", fd, kicked ? "（顶掉旧连接）" : "");
    }
  }

  // 返回 false = 连接已不可用
  bool read_client(int fd, std::string& rx_buf) {
    size_t budget = kMaxReadPerCycle;
    char chunk[4096];
    while (budget > 0) {
      const ssize_t got = ::recv(fd, chunk, sizeof(chunk), 0);
      if (got > 0) {
        rx_buf.append(chunk, static_cast<size_t>(got));
        budget -= static_cast<size_t>(got);
        continue;
      }
      if (got == 0) return false;  // 对端关闭
      if (errno == EAGAIN || errno == EWOULDBLOCK) return true;
      if (errno == EINTR) continue;
      return false;
    }
    return true;  // 本轮读满预算，回 poll 让位 accept
  }

  void split_lines(std::string& rx_buf) {
    size_t start = 0;
    for (;;) {
      const size_t nl = rx_buf.find('\n', start);
      if (nl == std::string::npos) break;
      handle_line(std::string_view(rx_buf).substr(start, nl - start));
      start = nl + 1;
    }
    if (start > 0) rx_buf.erase(0, start);
    if (rx_buf.size() > kMaxLineBytes) {  // 64 KiB 仍无换行：清空重同步
      rx_buf.clear();
      ++bad_lines_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "行缓冲超 64KiB 无换行，清空重同步（累计 %zu）", bad_lines_.load());
    }
  }

  void handle_line(std::string_view line) {
    // 整行 UTF-8 + JSON 合法性校验（nlohmann 解析器即校验两者）；
    // 字段语义一概不看，原始行原样上行（§4）。
    if (!nlohmann::json::accept(line.begin(), line.end())) {
      ++bad_lines_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "非法行丢弃（累计 %zu）",
                           bad_lines_.load());
      return;
    }
    std::lock_guard<std::mutex> lock(uplink_mu_);
    if (uplink_q_.size() >= kMaxQueuedLines) {
      ++uplink_dropped_;
      return;
    }
    uplink_q_.emplace(line);
  }

  void close_client(int& client_fd, std::string& rx_buf, std::string& out_buf,
                    const std::string& why) {
    if (client_fd < 0) return;
    RCLCPP_INFO(get_logger(), "客户端断开 fd=%d（%s）", client_fd, why.c_str());
    ::close(client_fd);
    client_fd = -1;
    rx_buf.clear();
    out_buf.clear();
  }

  void request_close(std::string why) {  // 仅 TCP 线程调用
    close_reason_ = std::move(why);
    close_requested_ = true;
  }

  void drain_tx(int client_fd, std::string& out_buf) {
    std::queue<std::string> batch;
    {
      std::lock_guard<std::mutex> lock(tx_mu_);
      batch.swap(tx_q_);
    }
    if (batch.empty()) return;
    if (client_fd < 0) {
      tx_dropped_ += batch.size();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "无客户端，下行丢弃 %zu 条（累计 %zu）", batch.size(),
                           tx_dropped_.load());
      return;
    }
    while (!batch.empty()) {
      out_buf += batch.front();
      out_buf += '\n';
      batch.pop();
    }
    if (out_buf.size() > kMaxOutBufBytes) {
      request_close("发送缓冲积压超 1MiB，踢掉慢客户端");
    }
  }

  void flush_out(int fd, std::string& out_buf) {
    while (!out_buf.empty()) {
      const ssize_t w = ::send(fd, out_buf.data(), out_buf.size(), MSG_NOSIGNAL);
      if (w > 0) {
        out_buf.erase(0, static_cast<size_t>(w));
        continue;
      }
      if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;  // 等下轮 POLLOUT
      if (w < 0 && errno == EINTR) continue;
      request_close(std::string("发送失败: ") + std::strerror(errno));
      return;
    }
  }

  // ---------- executor 线程 ----------

  void enqueue_tx(const std::string& line) {
    std::lock_guard<std::mutex> lock(tx_mu_);
    if (tx_q_.size() >= kMaxQueuedLines) {
      ++tx_dropped_;
      return;
    }
    tx_q_.push(line);
  }

  void drain_uplink() {
    std::queue<std::string> batch;
    {
      std::lock_guard<std::mutex> lock(uplink_mu_);
      batch.swap(uplink_q_);
    }
    while (!batch.empty()) {
      String msg;
      msg.data = std::move(batch.front());
      batch.pop();
      uplink_pub_->publish(msg);
      ++uplink_lines_;
    }
    if (const size_t dropped = uplink_dropped_.exchange(0)) {
      RCLCPP_WARN(get_logger(), "uplink 队列满，丢弃 %zu 行", dropped);
    }
  }

  int port_ = 10000;
  rclcpp::Publisher<String>::SharedPtr uplink_pub_;
  rclcpp::Subscription<String>::SharedPtr downlink_sub_;
  rclcpp::TimerBase::SharedPtr uplink_drain_;

  std::thread tcp_thread_;
  std::atomic<bool> running_{false};

  std::mutex uplink_mu_;
  std::queue<std::string> uplink_q_;
  std::mutex tx_mu_;
  std::queue<std::string> tx_q_;

  // TCP 线程私有
  int listen_fd_ = -1;
  bool close_requested_ = false;
  std::string close_reason_;
  size_t kicked_clients_ = 0;
  size_t uplink_lines_ = 0;  // executor 线程私有

  std::atomic<size_t> bad_lines_{0};
  std::atomic<size_t> uplink_dropped_{0};
  std::atomic<size_t> tx_dropped_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GatewayNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
