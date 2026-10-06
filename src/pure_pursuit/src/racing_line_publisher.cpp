#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "racecar_simulator/msg/speed_profile.hpp"

namespace {
struct RacingRow {
  double x{};
  double y{};
  double yaw{};
  double curvature{};
  double speed{};
  double steer{};
  double left_x{};
  double left_y{};
  double right_x{};
  double right_y{};
};

std::vector<std::string> split(const std::string &line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string t;
  while (std::getline(ss, t, ',')) {
    out.push_back(t);
  }
  return out;
}

std::string trim(const std::string &s) {
  size_t a = 0, b = s.size();
  while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
  return s.substr(a, b - a);
}

int findIdx(const std::vector<std::string> &headers, std::initializer_list<std::string> names) {
  for (auto &name : names) {
    for (size_t i = 0; i < headers.size(); ++i) {
      auto h = headers[i];
      std::string hn = trim(h);
      std::string nn = trim(name);
      std::transform(hn.begin(), hn.end(), hn.begin(), [](unsigned char c) { return std::tolower(c); });
      std::transform(nn.begin(), nn.end(), nn.begin(), [](unsigned char c) { return std::tolower(c); });
      if (hn == nn) return static_cast<int>(i);
    }
  }
  return -1;
}

bool parseDouble(const std::vector<std::string> &tok, int idx, double &out) {
  if (idx < 0 || idx >= static_cast<int>(tok.size())) return false;
  try {
    out = std::stod(tok[idx]);
    return true;
  } catch (...) {
    return false;
  }
}
}  // namespace

class RacingLinePublisher : public rclcpp::Node {
public:
  RacingLinePublisher() : Node("racing_line_publisher") {
    std::string default_csv;
    try {
      default_csv = ament_index_cpp::get_package_share_directory("racecar_simulator") +
                    std::string("/maps/f1tenth_racetracks/iccas2025/iccas2025_racing_profile.csv");
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "share dir lookup failed: %s", e.what());
    }

    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    local_path_topic_ = declare_parameter<std::string>("local_path_topic", "local_path");
    left_topic_ = declare_parameter<std::string>("left_boundary_topic", "racing_left");
    right_topic_ = declare_parameter<std::string>("right_boundary_topic", "racing_right");
    speed_profile_topic_ = declare_parameter<std::string>("speed_profile_topic", "speed_profile");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);
    racing_csv_path_ = declare_parameter<std::string>("racing_line_csv", default_csv);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    local_pub_ = create_publisher<nav_msgs::msg::Path>(local_path_topic_, qos);
    left_pub_ = create_publisher<nav_msgs::msg::Path>(left_topic_, qos);
    right_pub_ = create_publisher<nav_msgs::msg::Path>(right_topic_, qos);
    speed_pub_ = create_publisher<racecar_simulator::msg::SpeedProfile>(speed_profile_topic_, qos);

    if (!loadCsv()) {
      RCLCPP_FATAL(get_logger(), "Failed to load racing line csv: %s", racing_csv_path_.c_str());
      throw std::runtime_error("racing line missing");
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, publish_rate_hz_));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                               std::bind(&RacingLinePublisher::publishAll, this));

    RCLCPP_INFO(get_logger(), "Racing line loaded (%zu pts) from %s", rows_.size(),
                racing_csv_path_.c_str());
  }

private:
  bool loadCsv() {
    if (racing_csv_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "racing_line_csv param is empty");
      return false;
    }
    std::ifstream f(racing_csv_path_);
    if (!f.is_open()) {
      RCLCPP_ERROR(get_logger(), "Cannot open %s", racing_csv_path_.c_str());
      return false;
    }
    std::string line;
    if (!std::getline(f, line)) {
      RCLCPP_ERROR(get_logger(), "Empty csv: %s", racing_csv_path_.c_str());
      return false;
    }
    auto headers = split(line);
    int ix = findIdx(headers, {"x", "x_m"});
    int iy = findIdx(headers, {"y", "y_m"});
    int iyaw = findIdx(headers, {"yaw", "yaw_rad"});
    int ik = findIdx(headers, {"curvature", "kappa"});
    int ispeed = findIdx(headers, {"speed", "speed_mps"});
    int isteer = findIdx(headers, {"steer", "steering", "steer_rad"});
    int ilx = findIdx(headers, {"left_x", "left_x_m"});
    int ily = findIdx(headers, {"left_y", "left_y_m"});
    int irx = findIdx(headers, {"right_x", "right_x_m"});
    int iry = findIdx(headers, {"right_y", "right_y_m"});
    if (ix < 0 || iy < 0 || iyaw < 0 || ik < 0 || ispeed < 0 || isteer < 0 ||
        ilx < 0 || ily < 0 || irx < 0 || iry < 0) {
      RCLCPP_ERROR(get_logger(), "CSV missing required columns");
      return false;
    }
    rows_.clear();
    while (std::getline(f, line)) {
      if (line.empty()) continue;
      if (!line.empty() && line[0] == '#') continue;
      auto tok = split(line);
      RacingRow r{};
      if (!parseDouble(tok, ix, r.x) || !parseDouble(tok, iy, r.y)) continue;
      parseDouble(tok, iyaw, r.yaw);
      parseDouble(tok, ik, r.curvature);
      parseDouble(tok, ispeed, r.speed);
      parseDouble(tok, isteer, r.steer);
      parseDouble(tok, ilx, r.left_x);
      parseDouble(tok, ily, r.left_y);
      parseDouble(tok, irx, r.right_x);
      parseDouble(tok, iry, r.right_y);
      rows_.push_back(r);
    }
    buildMessages();
    return !rows_.empty();
  }

  void buildMessages() {
    nav_msgs::msg::Path p, l, r;
    p.header.frame_id = frame_id_;
    l.header.frame_id = frame_id_;
    r.header.frame_id = frame_id_;
    p.poses.reserve(rows_.size());
    l.poses.reserve(rows_.size());
    r.poses.reserve(rows_.size());
    profile_.speed.clear();
    profile_.steer.clear();
    profile_.speed.reserve(rows_.size());
    profile_.steer.reserve(rows_.size());

    for (const auto &row : rows_) {
      geometry_msgs::msg::PoseStamped c, left, right;
      c.header.frame_id = frame_id_;
      left.header.frame_id = frame_id_;
      right.header.frame_id = frame_id_;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, row.yaw);

      c.pose.position.x = row.x;
      c.pose.position.y = row.y;
      c.pose.orientation.x = q.x();
      c.pose.orientation.y = q.y();
      c.pose.orientation.z = q.z();
      c.pose.orientation.w = q.w();

      left.pose.position.x = row.left_x;
      left.pose.position.y = row.left_y;
      left.pose.orientation = c.pose.orientation;

      right.pose.position.x = row.right_x;
      right.pose.position.y = row.right_y;
      right.pose.orientation = c.pose.orientation;

      p.poses.push_back(c);
      l.poses.push_back(left);
      r.poses.push_back(right);
      profile_.speed.push_back(row.speed);
      profile_.steer.push_back(row.steer);
    }
    local_path_msg_ = std::move(p);
    left_msg_ = std::move(l);
    right_msg_ = std::move(r);
  }

  void publishAll() {
    const auto stamp = now();
    local_path_msg_.header.stamp = stamp;
    left_msg_.header.stamp = stamp;
    right_msg_.header.stamp = stamp;
    profile_.header.stamp = stamp;
    profile_.header.frame_id = frame_id_;
    local_pub_->publish(local_path_msg_);
    left_pub_->publish(left_msg_);
    right_pub_->publish(right_msg_);
    speed_pub_->publish(profile_);
  }

  std::string frame_id_;
  std::string local_path_topic_;
  std::string left_topic_;
  std::string right_topic_;
  std::string speed_profile_topic_;
  std::string racing_csv_path_;
  double publish_rate_hz_{};

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_pub_, left_pub_, right_pub_;
  rclcpp::Publisher<racecar_simulator::msg::SpeedProfile>::SharedPtr speed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<RacingRow> rows_;
  nav_msgs::msg::Path local_path_msg_, left_msg_, right_msg_;
  racecar_simulator::msg::SpeedProfile profile_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RacingLinePublisher>());
  rclcpp::shutdown();
  return 0;
}