#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "pure_pursuit/msg/obstacle_array.hpp"
#include "pure_pursuit/msg/obstacle_cluster.hpp"

class PerceptionNode : public rclcpp::Node {
public:
  PerceptionNode() : Node("perception_dbscan_node") {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan0", rclcpp::SensorDataQoS(),
        std::bind(&PerceptionNode::scanCallback, this, std::placeholders::_1));

    pub_ = create_publisher<pure_pursuit::msg::ObstacleArray>("obstacles", 10);

    eps_ = declare_parameter("eps", 0.14);
    min_pts_ = declare_parameter("min_pts", 4);
    front_angle_min_ = declare_parameter("front_angle_min", -1.3);
    front_angle_max_ = declare_parameter("front_angle_max", 1.3);
    min_range_ = declare_parameter("min_range", 0.05);
    max_range_ = declare_parameter("max_range", 8.0);
    lane_half_width_ = declare_parameter("lane_half_width", 0.65);
    wall_margin_ = declare_parameter("wall_margin", 0.0);   // 0 disables wall-as-obstacle widening
    front_block_angle_ = declare_parameter("front_block_angle", 0.35);
    front_block_range_ = declare_parameter("front_block_range", 1.2);
    x_min_ = declare_parameter("x_min", 0.0);
    x_max_ = declare_parameter("x_max", 4.5);
  }

private:
  double eps_{};
  int min_pts_{};
  double front_angle_min_{}, front_angle_max_{};
  double min_range_{}, max_range_{};
  double lane_half_width_{};
  double wall_margin_{};
  double front_block_angle_{};
  double front_block_range_{};
  double x_min_{}, x_max_{};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<pure_pursuit::msg::ObstacleArray>::SharedPtr pub_;

  struct Point {
    double x{}, y{};
    double angle{}, range{};
  };

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    auto pts = projectPoints(*scan);
    auto labels = dbscan(pts);
    bool front_block = detectFrontBlock(*scan);

    pure_pursuit::msg::ObstacleArray msg;
    msg.header = scan->header;

    std::unordered_map<int, std::vector<int>> clusters;
    for (int i = 0; i < static_cast<int>(labels.size()); ++i) {
      if (labels[i] >= 0) clusters[labels[i]].push_back(i);
    }

    for (auto &kv : clusters) {
      auto &idxs = kv.second;
      if (idxs.size() < static_cast<size_t>(min_pts_)) continue;

      pure_pursuit::msg::ObstacleCluster c;
      double sx = 0.0;
      double sy = 0.0;
      double max_r = 0.0;

      for (auto i : idxs) {
        sx += pts[i].x;
        sy += pts[i].y;
      }
      sx /= idxs.size();
      sy /= idxs.size();

      c.center.x = sx;
      c.center.y = sy;
      c.center.z = 0.0;

      for (auto i : idxs) {
        double dx = pts[i].x - sx;
        double dy = pts[i].y - sy;
        max_r = std::max(max_r, std::hypot(dx, dy));
      }

      c.radius = max_r;
      c.point_count = static_cast<uint32_t>(idxs.size());

      // 필터: 주행 차선/전방 영역 안에 있는 군집만 장애물로 간주
      if (c.center.x < x_min_ || c.center.x > x_max_) continue;
      const double allowed_y = lane_half_width_ + wall_margin_;
      if (std::abs(c.center.y) > allowed_y) continue;

      msg.clusters.push_back(c);
    }

    if (front_block) {
      pure_pursuit::msg::ObstacleCluster c;
      c.center.x = std::max(front_block_range_, x_min_);
      c.center.y = 0.0;
      c.center.z = 0.0;
      c.radius = 0.1;
      c.point_count = 1;
      msg.clusters.push_back(c);
    }

    pub_->publish(msg);
  }

  std::vector<Point> projectPoints(const sensor_msgs::msg::LaserScan &scan) {
    std::vector<Point> result;

    for (int i = 0; i < static_cast<int>(scan.ranges.size()); i++) {
      double angle = scan.angle_min + i * scan.angle_increment;

      if (angle < front_angle_min_ || angle > front_angle_max_) continue;

      double r = scan.ranges[i];
      if (!std::isfinite(r)) continue;
      if (r < min_range_ || r > max_range_) continue;

      Point p;
      p.range = r;
      p.angle = angle;
      p.x = r * std::cos(angle);
      p.y = r * std::sin(angle);

      result.push_back(p);
    }
    return result;
  }

  std::vector<int> dbscan(const std::vector<Point> &pts) {
    int n = static_cast<int>(pts.size());
    std::vector<int> lbl(n, -1);

    double eps2 = eps_ * eps_;
    int cluster_id = 0;

    for (int i = 0; i < n; i++) {
      if (lbl[i] != -1) continue;

      std::vector<int> neigh;
      for (int j = 0; j < n; j++) {
        double dx = pts[j].x - pts[i].x;
        double dy = pts[j].y - pts[i].y;
        if (dx * dx + dy * dy <= eps2) {
          neigh.push_back(j);
        }
      }

      if (static_cast<int>(neigh.size()) < min_pts_) {
        lbl[i] = -2;  // noise
        continue;
      }

      lbl[i] = cluster_id;

      std::queue<int> q;
      for (auto nidx : neigh) {
        if (nidx != i) q.push(nidx);
      }

      while (!q.empty()) {
        int cur = q.front();
        q.pop();

        if (lbl[cur] == -2) lbl[cur] = cluster_id;
        if (lbl[cur] != -1) continue;

        lbl[cur] = cluster_id;

        std::vector<int> neigh2;
        for (int j = 0; j < n; j++) {
          double dx = pts[j].x - pts[cur].x;
          double dy = pts[j].y - pts[cur].y;
          if (dx * dx + dy * dy <= eps2) {
            neigh2.push_back(j);
          }
        }

        if (static_cast<int>(neigh2.size()) >= min_pts_) {
          for (auto j : neigh2) {
            if (lbl[j] == -1) q.push(j);
          }
        }
      }

      cluster_id++;
    }

    return lbl;
  }

  bool detectFrontBlock(const sensor_msgs::msg::LaserScan &scan) const {
    double min_r = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(scan.ranges.size()); ++i) {
      double angle = scan.angle_min + i * scan.angle_increment;
      if (std::abs(angle) > front_block_angle_) continue;
      double r = scan.ranges[i];
      if (!std::isfinite(r)) continue;
      if (r < min_range_ || r > max_range_) continue;
      if (r < min_r) min_r = r;
    }
    return min_r < front_block_range_;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}