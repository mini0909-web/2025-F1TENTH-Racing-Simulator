#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "decision_ftg/first_order_filter.hpp"
#include "pure_pursuit/msg/obstacle_array.hpp"
#include "pure_pursuit/msg/obstacle_decision.hpp"

class DecisionFTG : public rclcpp::Node {
public:
  DecisionFTG() : Node("decision_ftg_node") {
    sub_ = create_subscription<pure_pursuit::msg::ObstacleArray>(
        "obstacles", 10, std::bind(&DecisionFTG::callback, this, std::placeholders::_1));

    pub_ = create_publisher<pure_pursuit::msg::ObstacleDecision>("obstacle_decision", 10);

    odom_topic_ = declare_parameter<std::string>("odom_topic", "odom0");
    slip_gain_ = declare_parameter<double>("slip_gain", 1.0);
    slip_lat_max_ = declare_parameter<double>("slip_lat_max", 0.35);
    memory_keep_time_ = declare_parameter<double>("obstacle_memory_time", 8.0);
    memory_merge_radius_ = declare_parameter<double>("obstacle_memory_merge_radius", 0.8);
    memory_penalty_scale_ = declare_parameter<double>("obstacle_memory_penalty_scale", 1.25);
    memory_speed_scale_ = declare_parameter<double>("obstacle_memory_speed_scale", 0.8);
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          vx_ = msg->twist.twist.linear.x;
          yaw_rate_ = msg->twist.twist.angular.z;
          odom_pose_ = msg->pose.pose;
          has_odom_ = true;
        });

    safe_dist_ = declare_parameter("safe_distance", 1.7);
    lane_width_ = declare_parameter("lane_width", 0.9);
    max_offset_ = declare_parameter("max_offset", 0.5);
    max_bias_ = declare_parameter("max_bias", 0.38);
    boundary_margin_ = declare_parameter("boundary_margin", 0.08);
    offset_filter_alpha_ = declare_parameter("offset_filter_alpha", 0.4);
    bias_filter_alpha_ = declare_parameter("bias_filter_alpha", 0.4);
    obstacle_slow_scale_ =
        declare_parameter("obstacle_slow_scale", 0.9);  // slight slowdown when obstacle exists
    offset_filter_.setAlpha(offset_filter_alpha_);
    bias_filter_.setAlpha(bias_filter_alpha_);
  }

private:
  rclcpp::Subscription<pure_pursuit::msg::ObstacleArray>::SharedPtr sub_;
  rclcpp::Publisher<pure_pursuit::msg::ObstacleDecision>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  std::string odom_topic_{};
  double slip_gain_{};
  double slip_lat_max_{};
  double memory_keep_time_{};
  double memory_merge_radius_{};
  double memory_penalty_scale_{};
  double memory_speed_scale_{};
  double vx_{0.0};
  double yaw_rate_{0.0};
  geometry_msgs::msg::Pose odom_pose_{};
  bool has_odom_{false};

  double safe_dist_{};
  double lane_width_{};
  double max_offset_{}, max_bias_{}, boundary_margin_{};
  double offset_filter_alpha_{};
  double bias_filter_alpha_{};
  double obstacle_slow_scale_{};
  FirstOrderFilter offset_filter_;
  FirstOrderFilter bias_filter_;
  struct PastObstacle {
    double x{};
    double y{};
    rclcpp::Time stamp;
  };
  std::vector<PastObstacle> memory_;

  static double yawFromQuat(const geometry_msgs::msg::Quaternion &q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  geometry_msgs::msg::Point globalFromLocal(double x, double y) const {
    geometry_msgs::msg::Point p;
    const auto &pose = odom_pose_;
    const double yaw = yawFromQuat(pose.orientation);
    const double cx = pose.position.x;
    const double cy = pose.position.y;
    p.x = cx + std::cos(yaw) * x - std::sin(yaw) * y;
    p.y = cy + std::sin(yaw) * x + std::cos(yaw) * y;
    p.z = 0.0;
    return p;
  }

  std::pair<double, double> localFromGlobal(const PastObstacle &m) const {
    const auto &pose = odom_pose_;
    const double yaw = yawFromQuat(pose.orientation);
    const double dx = m.x - pose.position.x;
    const double dy = m.y - pose.position.y;
    double x_local = std::cos(yaw) * dx + std::sin(yaw) * dy;
    double y_local = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    return {x_local, y_local};
  }

  void addOrRefreshMemory(const geometry_msgs::msg::Point &p, const rclcpp::Time &stamp) {
    for (auto &m : memory_) {
      double dx = m.x - p.x;
      double dy = m.y - p.y;
      if (std::hypot(dx, dy) <= memory_merge_radius_) {
        m.x = 0.5 * (m.x + p.x);
        m.y = 0.5 * (m.y + p.y);
        m.stamp = stamp;
        return;
      }
    }
    memory_.push_back({p.x, p.y, stamp});
  }

  void pruneMemory(const rclcpp::Time &now_time) {
    const double keep = memory_keep_time_;
    memory_.erase(std::remove_if(memory_.begin(), memory_.end(), [&](const PastObstacle &m) {
                    return (now_time - m.stamp).seconds() > keep;
                  }),
                  memory_.end());
  }

  void callback(const pure_pursuit::msg::ObstacleArray::SharedPtr msg) {
    pure_pursuit::msg::ObstacleDecision out;
    out.header = msg->header;

    if (msg->clusters.empty()) {
      offset_filter_.reset(0.0);
      bias_filter_.reset(0.0);
      out.lateral_offset = 0.0;
      out.steering_bias = 0.0;
      out.speed_scale = 1.0;
      out.obstacle_active = false;
      pub_->publish(out);
      return;
    }

    double left_penalty = 0.0;
    double right_penalty = 0.0;

    bool danger = false;

    const auto now_time = now();
    pruneMemory(now_time);

    for (auto &c : msg->clusters) {
      double x_local = c.center.x;
      double y_local = c.center.y;

      if (x_local < 0.0 || x_local > 6.0) continue;  // 앞쪽만 고려

      double slip_lat = 0.0;
      if (has_odom_ && std::abs(yaw_rate_) > 1e-5) {
        const double v = std::max(vx_, 0.1);
        const double lat = slip_gain_ * yaw_rate_ * x_local * x_local / (2.0 * v);
        slip_lat = std::clamp(lat, -slip_lat_max_, slip_lat_max_);
      }
      double y_path = y_local - slip_lat;  // 예상 궤적 대비 횡오차

      double dist = std::hypot(x_local, y_path);
      if (dist < safe_dist_) danger = true;

      if (std::abs(y_path) < lane_width_) {
        if (y_path >= 0) {
          left_penalty += 1.0 / (x_local + 0.3);
        } else {
          right_penalty += 1.0 / (x_local + 0.3);
        }
      }

      if (has_odom_) {
        addOrRefreshMemory(globalFromLocal(x_local, y_local), now_time);
      }
    }

    if (has_odom_) {
      for (const auto &m : memory_) {
        auto [x_local, y_local] = localFromGlobal(m);
        if (x_local < 0.0 || x_local > 6.0) continue;

        double slip_lat = 0.0;
        if (std::abs(yaw_rate_) > 1e-5) {
          const double v = std::max(vx_, 0.1);
          const double lat = slip_gain_ * yaw_rate_ * x_local * x_local / (2.0 * v);
          slip_lat = std::clamp(lat, -slip_lat_max_, slip_lat_max_);
        }
        double y_path = y_local - slip_lat;

        double dist = std::hypot(x_local, y_path);
        if (dist < safe_dist_) danger = true;

        if (std::abs(y_path) < lane_width_) {
          const double weight = memory_penalty_scale_;
          if (y_path >= 0) {
            left_penalty += weight / (x_local + 0.3);
          } else {
            right_penalty += weight / (x_local + 0.3);
          }
        }
      }
    }

    double side = right_penalty - left_penalty;  // +면 왼쪽으로 회피
    double offset = std::clamp(side * 1.0, -max_offset_, max_offset_);
    // 실측 장애물 존재 시에는 바로 최대 오프셋으로 비켜가기
    if (!msg->clusters.empty()) {
      const double dir = std::abs(side) > 1e-6 ? std::copysign(1.0, side) : 1.0;  // 기본은 우측
      offset = std::clamp(dir * max_offset_, -max_offset_, max_offset_);
    }

    // 차선 경계 여유를 고려해 한 번 더 클램프
    const double offset_limit = std::max(0.0, lane_width_ - boundary_margin_);
    offset = std::clamp(offset, -offset_limit, offset_limit);

    double bias = std::clamp(offset * 0.35, -max_bias_, max_bias_);

    const bool bypass_filter = danger;
    double offset_filtered = bypass_filter ? offset : offset_filter_.apply(offset);
    double bias_filtered = bypass_filter ? bias : bias_filter_.apply(bias);

    out.lateral_offset = offset_filtered;
    out.steering_bias = bias_filtered;

    const bool memory_only_active = !msg->clusters.empty() ? false : danger;
    out.obstacle_active = (std::abs(offset_filtered) > 0.05) || memory_only_active;

    const double active_slow = 0.5;  // 장애물 있을 때 기본 반속
    if (danger) {
      out.speed_scale = std::min(active_slow, 0.45);  // 더 강하게 감속
    } else if (memory_only_active) {
      out.speed_scale = std::min({active_slow, obstacle_slow_scale_, memory_speed_scale_});
    } else {
      out.speed_scale = out.obstacle_active ? std::min(active_slow, obstacle_slow_scale_) : 1.0;
    }

    pub_->publish(out);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DecisionFTG>());
  rclcpp::shutdown();
  return 0;
}
