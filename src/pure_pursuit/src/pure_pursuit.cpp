// Pure pursuit with curvature-aware feedforward, obstacle bias, and speed profile lookup.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <mutex>
#include <string>
#include <utility>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/u_int32.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "racecar_simulator/msg/speed_profile.hpp"
#include "pure_pursuit/msg/obstacle_decision.hpp"

class PurePursuitNode : public rclcpp::Node {
public:
  PurePursuitNode() : Node("pure_pursuit_node") {
    lookahead_ = declare_parameter<double>("lookahead", 1.5);
    lookahead_speed_gain_ = declare_parameter<double>("lookahead_speed_gain", 0.05);
    lookahead_min_ = declare_parameter<double>("lookahead_min", 0.7);
    lookahead_max_ = declare_parameter<double>("lookahead_max", 1.3);
    lookahead_curv_gain_ = declare_parameter<double>("lookahead_curvature_gain", 0.8);
    wheelbase_ = declare_parameter<double>("wheelbase", 0.34);
    v_min_ = declare_parameter<double>("speed_min", 5.2);
    v_max_ = declare_parameter<double>("speed_max", 10.0);
    k_accel_ = declare_parameter<double>("k_accel", 2.8);
    a_min_ = declare_parameter<double>("accel_min", -5.8);
    a_max_ = declare_parameter<double>("accel_max", 2.1);
    Kus_ = declare_parameter<double>("Kus", 0.0);
    K_yaw_ = declare_parameter<double>("K_yaw", 0.045);
    K_corr_ = declare_parameter<double>("K_corr", 0.95);
    mu_ = declare_parameter<double>("mu", 1.0);
    max_steer_ = declare_parameter<double>("max_steer", 0.45);
    avoid_steer_gain_ =
        declare_parameter<double>("avoid_steer_gain", 1.2);  // allow larger steer when avoiding
    max_corr_offset_ = declare_parameter<double>("max_corr_offset", 0.3);
    obstacle_max_corr_offset_ =
        declare_parameter<double>("obstacle_max_corr_offset", 0.60);  // allow larger dodge on obstacles
    boundary_margin_ = declare_parameter<double>("boundary_margin", 0.14);
    curvature_speed_gain_gentle_ =
        declare_parameter<double>("curvature_speed_gain_gentle", 0.75);  // mild curves
    curvature_speed_gain_sharp_ =
        declare_parameter<double>("curvature_speed_gain_sharp", 3.5);    // sharp curves
    curvature_speed_breakpoint_ =
        declare_parameter<double>("curvature_speed_breakpoint", 0.45);   // |kappa| where sharp kicks in
    curve_slowdown_steer_ref_ = declare_parameter<double>("curve_slowdown_steer_ref", 0.4);
    curve_slowdown_gain_ = declare_parameter<double>("curve_slowdown_gain", 1.7);
    curve_slowdown_min_scale_ = declare_parameter<double>("curve_slowdown_min_scale", 0.45);
    curve_bias_window_size_ = declare_parameter<int>("curve_bias_window_size", 7);
    curve_bias_curvature_thresh_ =
        declare_parameter<double>("curve_bias_curvature_thresh", 0.7);  // treat as sharp sequence
    curve_bias_consistency_thresh_ =
        declare_parameter<double>("curve_bias_consistency_thresh", 0.6);  // same-turn ratio
    curve_bias_offset_gain_ =
        declare_parameter<double>("curve_bias_offset_gain", 0.15);  // inward bias (m) on sharp runs
    curve_bias_lookahead_scale_ = declare_parameter<double>("curve_bias_lookahead_scale", 0.85);
    max_steer_sharp_ = declare_parameter<double>("max_steer_sharp", 0.45);
    obstacle_curve_curv_thresh_ =
        declare_parameter<double>("obstacle_curve_curvature_thresh", 0.8);  // extra braking when |kappa| large
    obstacle_curve_speed_scale_ =
        declare_parameter<double>("obstacle_curve_speed_scale", 0.55);      // strong slowdown factor
    obstacle_curve_curv_scale_ =
        declare_parameter<double>("obstacle_curve_curv_scale", 0.9);        // soften curvature when avoiding
    avoid_feedforward_gain_ = declare_parameter<double>("avoid_feedforward_gain", 1.35);
    avoid_feedback_gain_ = declare_parameter<double>("avoid_feedback_gain", 0.65);
    avoid_offset_filter_alpha_ = declare_parameter<double>("avoid_offset_filter_alpha", 0.25);
    avoid_return_rate_ = declare_parameter<double>("avoid_return_rate", 0.65);
    obstacle_persist_time_ = declare_parameter<double>("obstacle_persist_time", 0.6);
    avoid_max_offset_ = declare_parameter<double>("avoid_max_offset", 0.65);
    obstacle_offset_trigger_ = declare_parameter<double>("obstacle_offset_trigger", 0.015);
    local_path_topic_ = declare_parameter<std::string>("local_path_topic", "local_path");
    left_path_topic_ = declare_parameter<std::string>("left_boundary", "left_boundary");
    right_path_topic_ = declare_parameter<std::string>("right_boundary", "right_boundary");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "odom0");
    drive_topic_ = declare_parameter<std::string>("drive_topic", "ackermann_cmd0");
    speed_profile_topic_ = declare_parameter<std::string>("speed_profile_topic", "speed_profile");
    index_topic_ = declare_parameter<std::string>("path_index_topic", "path_index");

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    drive_pub_ =
        create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, pub_qos);
    index_pub_ = create_publisher<std_msgs::msg::UInt32>(index_topic_, pub_qos);

    sub_local_ = create_subscription<nav_msgs::msg::Path>(
        local_path_topic_, map_qos,
        [this](const nav_msgs::msg::Path::SharedPtr msg) { local_path_ = *msg; });

    sub_left_ = create_subscription<nav_msgs::msg::Path>(
        left_path_topic_, map_qos,
        [this](const nav_msgs::msg::Path::SharedPtr msg) { left_path_ = *msg; });

    sub_right_ = create_subscription<nav_msgs::msg::Path>(
        right_path_topic_, map_qos,
        [this](const nav_msgs::msg::Path::SharedPtr msg) { right_path_ = *msg; });

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, sub_qos, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          odom_ = *msg;
          has_odom_ = true;
        });

    speed_profile_sub_ = create_subscription<racecar_simulator::msg::SpeedProfile>(
        speed_profile_topic_, map_qos,
        std::bind(&PurePursuitNode::speedProfileCallback, this, std::placeholders::_1));

    decision_sub_ = create_subscription<pure_pursuit::msg::ObstacleDecision>(
        "obstacle_decision", 10,
        std::bind(&PurePursuitNode::decisionCallback, this, std::placeholders::_1));
    latest_decision_.speed_scale = 1.0;

    timer_ = create_wall_timer(std::chrono::milliseconds(10),
                               std::bind(&PurePursuitNode::onTimer, this));

    // Initialize time stamps with the node's clock so later differences share the same source.
    const auto t0 = now();
    last_obstacle_time_ = t0 - rclcpp::Duration::from_seconds(obstacle_persist_time_);
    last_timer_time_ = t0;
  }

private:
  enum class PathMode { FOLLOW_LOCAL, AVOIDING, RETURNING };

  struct CurvatureWindowInfo {
    double avg_abs{0.0};
    double signed_mean{0.0};
    double sign_consistency{0.0};
  };

  void onTimer() {
    if (!has_odom_) return;
    const bool has_local_path = !local_path_.poses.empty();
    if (!has_local_path) {
      if (!local_path_warned_) {
        RCLCPP_WARN(get_logger(), "local_path is empty (check your CSV)");
        local_path_warned_ = true;
      }
      return;
    }
    local_path_warned_ = false;
    const nav_msgs::msg::Path *active_path = &local_path_;

    const auto now_time = now();
    if (last_obstacle_time_.get_clock_type() != now_time.get_clock_type()) {
      last_obstacle_time_ = now_time;
    }
    double dt = 0.01;
    if (last_timer_valid_) {
      dt = (now_time - last_timer_time_).seconds();
      if (dt <= 0.0) dt = 0.01;
    }
    last_timer_time_ = now_time;
    last_timer_valid_ = true;

    const bool decision_has_offset = std::abs(latest_decision_.lateral_offset) >= obstacle_offset_trigger_;
    if (latest_decision_.obstacle_active || decision_has_offset) {
      last_obstacle_time_ = now_time;
    }
    const bool obstacle_recent =
        (now_time - last_obstacle_time_).seconds() <= obstacle_persist_time_;
    const double active_offset_hint =
        std::max(std::abs(latest_decision_.lateral_offset), std::abs(avoidance_target_offset_));

    const auto &p = odom_.pose.pose.position;
    const auto &q = odom_.pose.pose.orientation;
    double roll, pitch, yaw;
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
    const double x = p.x;
    const double y = p.y;

    const double vx = odom_.twist.twist.linear.x;
    double dynamic_lookahead =
        std::clamp(lookahead_ + lookahead_speed_gain_ * std::max(vx, 0.0), lookahead_min_,
                   lookahead_max_);
    int target_idx = findLookaheadIndexCyclic(*active_path, x, y, dynamic_lookahead);
    if (target_idx < 0) return;
    last_target_index_ = static_cast<size_t>(target_idx);
    publishIndex(last_target_index_);

    const auto &tp = active_path->poses[last_target_index_].pose.position;
    const double path_curvature = computePathCurvature(*active_path, last_target_index_);
    auto curve_window =
        curvatureWindowStats(*active_path, last_target_index_, curve_bias_window_size_);
    bool sharp_sequence = curve_window.avg_abs >= curve_bias_curvature_thresh_ &&
                          curve_window.sign_consistency >= curve_bias_consistency_thresh_;
    double curve_dir = sharp_sequence ? (curve_window.signed_mean >= 0.0 ? 1.0 : -1.0) : 0.0;
    double sharp_intensity =
        sharp_sequence ? std::clamp((curve_window.avg_abs - curve_bias_curvature_thresh_) /
                                        (curve_bias_curvature_thresh_ + 1e-6),
                                    0.0, 1.0)
                       : 0.0;
    double curv_used = path_curvature;
    const double kappa_orig = std::abs(path_curvature);
    const bool avoidance_active_for_curve = obstacle_recent || path_mode_ != PathMode::FOLLOW_LOCAL;
    const bool obstacle_in_sharp_curve = avoidance_active_for_curve &&
                                         active_offset_hint > 0.05 &&
                                         kappa_orig >= obstacle_curve_curv_thresh_;
    if (obstacle_in_sharp_curve) {
      curv_used *= obstacle_curve_curv_scale_;
    }
    // 룩어헤드는 원래 경로 곡률을 기준으로 줄여, 급커브에서도 가까운 목표를 보도록 한다.
    const double curv_for_lookahead = obstacle_in_sharp_curve ? path_curvature : curv_used;
    double lookahead_curv =
        std::clamp(dynamic_lookahead - lookahead_curv_gain_ * std::abs(curv_for_lookahead),
                   lookahead_min_, lookahead_max_);
    if (sharp_sequence) {
      lookahead_curv = std::max(lookahead_min_, lookahead_curv * curve_bias_lookahead_scale_);
    }
    if (std::abs(lookahead_curv - dynamic_lookahead) > 1e-3) {
      int target_idx2 = findLookaheadIndexCyclic(*active_path, x, y, lookahead_curv);
      if (target_idx2 >= 0) {
        target_idx = target_idx2;
        last_target_index_ = static_cast<size_t>(target_idx);
        publishIndex(last_target_index_);
        dynamic_lookahead = lookahead_curv;
      }
    }
    curve_window =
        curvatureWindowStats(*active_path, last_target_index_, curve_bias_window_size_);
    sharp_sequence = curve_window.avg_abs >= curve_bias_curvature_thresh_ &&
                     curve_window.sign_consistency >= curve_bias_consistency_thresh_;
    curve_dir = sharp_sequence ? (curve_window.signed_mean >= 0.0 ? 1.0 : -1.0) : 0.0;
    sharp_intensity =
        sharp_sequence ? std::clamp((curve_window.avg_abs - curve_bias_curvature_thresh_) /
                                        (curve_bias_curvature_thresh_ + 1e-6),
                                    0.0, 1.0)
                       : 0.0;
    const double yaw_meas = odom_.twist.twist.angular.z;
    const double delta_ff_base = wheelbase_ * curv_used + Kus_ * vx * vx * curv_used;
    const double steer_profile = lookupProfileSteer(last_target_index_, wheelbase_ * curv_used);
    const double delta_profile = steer_profile - wheelbase_ * curv_used;
    const double delta_ff = delta_ff_base + delta_profile;
    const double yaw_ref = vx * curv_used;
    const double yaw_err = yaw_ref - yaw_meas;
    const double delta_stab = K_yaw_ * yaw_err;
    const double yL_center = -std::sin(yaw) * (tp.x - x) + std::cos(yaw) * (tp.y - y);

    // 장애물 회피용 피드포워드/피드백 오프셋 계산
    const double obstacle_offset_ff =
        std::clamp(latest_decision_.lateral_offset * avoid_feedforward_gain_, -avoid_max_offset_,
                   avoid_max_offset_);

    updateAvoidanceState(obstacle_recent, obstacle_offset_ff, dt);

    double avoidance_cmd = 0.0;
    if (path_mode_ != PathMode::FOLLOW_LOCAL || std::abs(avoidance_target_offset_) > 1e-3) {
      double fb_term = avoid_feedback_gain_ * (avoidance_target_offset_ - yL_center);
      avoidance_cmd = avoid_feedforward_gain_ * avoidance_target_offset_ + fb_term;
    }
    avoidance_cmd = filterAvoidanceOffset(avoidance_cmd);

    if (obstacle_recent) {
      const int mode_i = static_cast<int>(path_mode_);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                           "avoid mode=%d target=%.3f fb=%.3f cmd=%.3f lateral=%.3f",
                           mode_i, avoidance_target_offset_, avoidance_cmd - avoid_feedforward_gain_ * avoidance_target_offset_,
                           avoidance_cmd, latest_decision_.lateral_offset);
    }

    const double offset_raw = K_corr_ * (delta_ff + delta_stab) + avoidance_cmd;
    const bool avoiding_now = path_mode_ != PathMode::FOLLOW_LOCAL;
    const double offset_limit = avoiding_now ? obstacle_max_corr_offset_ : max_corr_offset_;
    double offset = std::clamp(offset_raw, -offset_limit, offset_limit);
    if (sharp_sequence && std::abs(curve_dir) > 1e-6) {
      const double bias_scale = 0.5 + 0.5 * sharp_intensity;
      offset += curve_dir * curve_bias_offset_gain_ * bias_scale;
    }
    const auto normal = computeNormal(*active_path, last_target_index_);
    offset = clampToBoundaries(offset, last_target_index_, *active_path, left_path_, right_path_,
                               normal, boundary_margin_);

    const double corrected_x = tp.x + offset * normal.first;
    const double corrected_y = tp.y + offset * normal.second;

    const double dx = corrected_x - x;
    const double dy = corrected_y - y;
    const double xL = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double yL = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    if (xL <= 0.01) return;

    const double Ld = std::hypot(xL, yL);
    const double curvature_pp = 2.0 * yL / (Ld * Ld);
    const double steer_pp = std::atan(wheelbase_ * curvature_pp);

    double steer_final = steer_pp + delta_ff + delta_stab;
    steer_final += latest_decision_.steering_bias;
    double steer_limit = max_steer_;
    if (avoiding_now) {
      // 장애물 회피 시에는 의도적으로 더 큰 조타를 허용한다.
      steer_limit = std::max(steer_limit, max_steer_ * std::max(avoid_steer_gain_, 1.0));
    }
    if (sharp_sequence && max_steer_sharp_ > max_steer_) {
      steer_limit = max_steer_sharp_;
    }
    if (steer_final > steer_limit) steer_final = steer_limit;
    if (steer_final < -steer_limit) steer_final = -steer_limit;

    double v_ref = lookupProfileSpeed(last_target_index_, curv_used);
    const double kappa = std::abs(curv_used);
    const double gentle_term = curvature_speed_gain_gentle_ * std::min(kappa, curvature_speed_breakpoint_);
    const double sharp_term =
        curvature_speed_gain_sharp_ * std::max(kappa - curvature_speed_breakpoint_, 0.0);
    double curv_scale = 1.0 / (1.0 + gentle_term + sharp_term);
    v_ref = std::max(0.0, v_ref * curv_scale * latest_decision_.speed_scale);

    // 장애물 + 급커브에서는 추가로 크게 감속해 차선을 확실히 바꿔 지나가도록 한다.
    if (obstacle_in_sharp_curve) {
      v_ref *= obstacle_curve_speed_scale_;
    }

    // 추가 감속: 큰 스티어(급커브)에서는 속도를 더 줄인다.
    const double steer_abs = std::abs(steer_final);
    const double steer_band = std::max(max_steer_ - curve_slowdown_steer_ref_, 1e-3);
    const double steer_excess = std::max(0.0, steer_abs - curve_slowdown_steer_ref_);
    const double steer_ratio = steer_excess / steer_band;
    double steer_scale = 1.0 / (1.0 + curve_slowdown_gain_ * steer_ratio);
    steer_scale = std::clamp(steer_scale, curve_slowdown_min_scale_, 1.0);
    v_ref *= steer_scale;

    const double v = odom_.twist.twist.linear.x;
    double a_cmd = k_accel_ * (v_ref - v);
    if (a_cmd > a_max_) a_cmd = a_max_;
    if (a_cmd < a_min_) a_cmd = a_min_;

    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";
    cmd.drive.steering_angle = steer_final;
    cmd.drive.acceleration = a_cmd;
    drive_pub_->publish(cmd);
  }

  static int findLookaheadIndexCyclic(const nav_msgs::msg::Path &path, double x, double y,
                                      double Ld) {
    const size_t N = path.poses.size();
    if (N == 0) return -1;
    if (N == 1) return 0;
    size_t closest = 0;
    double best_d2 = std::numeric_limits<double>::max();
    for (size_t i = 0; i < N; ++i) {
      const auto &pt = path.poses[i].pose.position;
      double d2 = (pt.x - x) * (pt.x - x) + (pt.y - y) * (pt.y - y);
      if (d2 < best_d2) {
        best_d2 = d2;
        closest = i;
      }
    }

    double accum = 0.0;
    for (size_t step = 0; step < N; ++step) {
      size_t i = (closest + step) % N;
      size_t inext = (i + 1) % N;
      const auto &a = path.poses[i].pose.position;
      const auto &b = path.poses[inext].pose.position;
      accum += std::hypot(b.x - a.x, b.y - a.y);
      if (accum >= Ld) return static_cast<int>(inext);
    }

    return static_cast<int>(closest);
  }

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr index_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_local_, sub_left_, sub_right_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<pure_pursuit::msg::ObstacleDecision>::SharedPtr decision_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path local_path_, left_path_, right_path_;
  nav_msgs::msg::Odometry odom_;
  bool has_odom_{false};

  double lookahead_, lookahead_speed_gain_, lookahead_min_, lookahead_max_, lookahead_curv_gain_;
  double wheelbase_;
  double v_min_, v_max_;
  double k_accel_, a_min_, a_max_;
  double Kus_, K_yaw_, K_corr_, mu_, max_steer_, avoid_steer_gain_, max_corr_offset_, obstacle_max_corr_offset_,
      boundary_margin_;
  double curvature_speed_gain_gentle_, curvature_speed_gain_sharp_, curvature_speed_breakpoint_;
  double curve_slowdown_steer_ref_, curve_slowdown_gain_, curve_slowdown_min_scale_;
  int curve_bias_window_size_;
  double curve_bias_curvature_thresh_, curve_bias_consistency_thresh_, curve_bias_offset_gain_;
  double curve_bias_lookahead_scale_, max_steer_sharp_;
  double obstacle_curve_curv_thresh_, obstacle_curve_speed_scale_, obstacle_curve_curv_scale_;
  double avoid_feedforward_gain_, avoid_feedback_gain_, avoid_offset_filter_alpha_;
  double avoid_return_rate_, obstacle_persist_time_, avoid_max_offset_;
  double obstacle_offset_trigger_;
  std::string local_path_topic_, left_path_topic_, right_path_topic_;
  std::string odom_topic_, drive_topic_, speed_profile_topic_, index_topic_;

  rclcpp::Subscription<racecar_simulator::msg::SpeedProfile>::SharedPtr speed_profile_sub_;
  racecar_simulator::msg::SpeedProfile::SharedPtr speed_profile_;
  pure_pursuit::msg::ObstacleDecision latest_decision_;
  mutable std::mutex profile_mutex_;
  size_t last_target_index_{0};
  bool local_path_warned_{false};
  PathMode path_mode_{PathMode::FOLLOW_LOCAL};
  double avoidance_target_offset_{0.0};
  double avoidance_filtered_offset_{0.0};
  bool avoidance_filter_init_{false};
  rclcpp::Time last_obstacle_time_{};
  rclcpp::Time last_timer_time_{};
  bool last_timer_valid_{false};

  void decisionCallback(const pure_pursuit::msg::ObstacleDecision::SharedPtr msg) {
    latest_decision_ = *msg;
    if (msg->obstacle_active) {
      last_obstacle_time_ = now();
    }
  }

  void speedProfileCallback(const racecar_simulator::msg::SpeedProfile::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(profile_mutex_);
    speed_profile_ = msg;
  }

  void publishIndex(size_t idx) {
    if (!index_pub_) return;
    std_msgs::msg::UInt32 msg;
    msg.data = static_cast<uint32_t>(idx);
    index_pub_->publish(msg);
  }

  void updateAvoidanceState(bool obstacle_recent, double obstacle_offset_ff, double dt) {
    if (obstacle_recent) {
      const double target = std::clamp(obstacle_offset_ff, -avoid_max_offset_, avoid_max_offset_);
      if (path_mode_ == PathMode::FOLLOW_LOCAL) {
        path_mode_ = PathMode::AVOIDING;
        avoidance_target_offset_ = target;
      } else if (path_mode_ == PathMode::AVOIDING) {
        const double a = avoid_offset_filter_alpha_;
        avoidance_target_offset_ = a * avoidance_target_offset_ + (1.0 - a) * target;
      } else if (path_mode_ == PathMode::RETURNING) {
        path_mode_ = PathMode::AVOIDING;
        avoidance_target_offset_ = target;
      }
    } else {
      if (path_mode_ == PathMode::AVOIDING) {
        path_mode_ = PathMode::RETURNING;
      }

      if (path_mode_ == PathMode::RETURNING) {
        const double step = avoid_return_rate_ * std::max(dt, 0.0);
        if (std::abs(avoidance_target_offset_) <= step) {
          avoidance_target_offset_ = 0.0;
          path_mode_ = PathMode::FOLLOW_LOCAL;
        } else {
          avoidance_target_offset_ -= std::copysign(step, avoidance_target_offset_);
        }
      }
    }

    avoidance_target_offset_ =
        std::clamp(avoidance_target_offset_, -avoid_max_offset_, avoid_max_offset_);
  }

  double filterAvoidanceOffset(double cmd) {
    if (!avoidance_filter_init_) {
      avoidance_filtered_offset_ = cmd;
      avoidance_filter_init_ = true;
      return avoidance_filtered_offset_;
    }
    const double a = avoid_offset_filter_alpha_;
    avoidance_filtered_offset_ = a * cmd + (1.0 - a) * avoidance_filtered_offset_;
    return avoidance_filtered_offset_;
  }

  double lookupProfileSpeed(size_t target_index, double curvature) const {
    std::lock_guard<std::mutex> lock(profile_mutex_);
    const double fallback = curvatureFallback(curvature);
    if (!speed_profile_ || speed_profile_->speed.empty()) return fallback;
    const size_t n = speed_profile_->speed.size();
    if (n == 0) return fallback;
    const size_t idx = target_index % n;
    double profile_speed = speed_profile_->speed[idx];
    if (profile_speed <= 0.0) return fallback;
    return std::clamp(profile_speed, v_min_, v_max_);
  }

  double lookupProfileSteer(size_t target_index, double fallback) const {
    std::lock_guard<std::mutex> lock(profile_mutex_);
    if (!speed_profile_ || speed_profile_->steer.empty()) return fallback;
    const size_t n = speed_profile_->steer.size();
    if (n == 0) return fallback;
    double steer = speed_profile_->steer[target_index % n];
    if (!std::isfinite(steer)) return fallback;
    return std::clamp(steer, -max_steer_, max_steer_);
  }

  double curvatureFallback(double curvature) const {
    const double denom = std::abs(curvature) + 1e-3;
    double v_kappa = std::sqrt(std::max(mu_ * 9.81 / denom, 0.0));
    if (!std::isfinite(v_kappa)) v_kappa = v_min_;
    return std::clamp(v_kappa, v_min_, v_max_);
  }

  static double computePathCurvature(const nav_msgs::msg::Path &path, size_t idx) {
    const size_t N = path.poses.size();
    if (N < 3) return 0.0;
    const size_t prev = idx == 0 ? N - 1 : idx - 1;
    const size_t next = (idx + 1) % N;
    const auto &pa = path.poses[prev].pose.position;
    const auto &pb = path.poses[idx].pose.position;
    const auto &pc = path.poses[next].pose.position;
    const double ab = std::hypot(pb.x - pa.x, pb.y - pa.y);
    const double bc = std::hypot(pc.x - pb.x, pc.y - pb.y);
    const double ca = std::hypot(pa.x - pc.x, pa.y - pc.y);
    const double denom = ab * bc * ca;
    if (denom < 1e-6) return 0.0;
    const double area2 = (pb.x - pa.x) * (pc.y - pa.y) - (pb.y - pa.y) * (pc.x - pa.x);
    return 2.0 * area2 / denom;
  }

  static std::pair<double, double> computeNormal(const nav_msgs::msg::Path &path, size_t idx) {
    const size_t N = path.poses.size();
    if (N < 2) return {0.0, 1.0};
    const size_t prev = idx == 0 ? N - 1 : idx - 1;
    const size_t next = (idx + 1) % N;
    const auto &pa = path.poses[prev].pose.position;
    const auto &pc = path.poses[next].pose.position;
    double tx = pc.x - pa.x;
    double ty = pc.y - pa.y;
    const double norm = std::hypot(tx, ty);
    if (norm < 1e-6) return {0.0, 1.0};
    tx /= norm;
    ty /= norm;
    return {-ty, tx};
  }

  static double clampToBoundaries(double offset, size_t idx, const nav_msgs::msg::Path &center,
                                  const nav_msgs::msg::Path &left,
                                  const nav_msgs::msg::Path &right,
                                  const std::pair<double, double> &normal, double margin) {
    if (center.poses.empty()) return offset;
    const auto &c = center.poses[idx % center.poses.size()].pose.position;

    double left_allow = std::numeric_limits<double>::infinity();
    double right_allow = std::numeric_limits<double>::infinity();

    if (!left.poses.empty()) {
      const auto &l = left.poses[idx % left.poses.size()].pose.position;
      const double dl = (l.x - c.x) * normal.first + (l.y - c.y) * normal.second;
      left_allow = std::max(0.0, dl - margin);
    }

    if (!right.poses.empty()) {
      const auto &r = right.poses[idx % right.poses.size()].pose.position;
      const double dr = (r.x - c.x) * normal.first + (r.y - c.y) * normal.second;
      right_allow = std::max(0.0, -dr - margin);  // normal은 좌측을 +로 가정
    }

    if (!std::isfinite(left_allow) && !std::isfinite(right_allow)) return offset;

    double min_left = std::isfinite(left_allow) ? left_allow : std::numeric_limits<double>::infinity();
    double min_right = std::isfinite(right_allow) ? right_allow : std::numeric_limits<double>::infinity();

    double clamped = offset;
    clamped = std::min(clamped, min_left);
    clamped = std::max(clamped, -min_right);
    return clamped;
  }

  CurvatureWindowInfo curvatureWindowStats(const nav_msgs::msg::Path &path, size_t start_idx,
                                           int window) const {
    CurvatureWindowInfo info{};
    if (window <= 0) return info;
    const size_t N = path.poses.size();
    if (N < 3) return info;
    const size_t W = static_cast<size_t>(window);
    double abs_sum = 0.0;
    double signed_sum = 0.0;
    size_t count = 0;
    for (size_t i = 0; i < W; ++i) {
      const size_t idx = (start_idx + i) % N;
      double k = computePathCurvature(path, idx);
      if (!std::isfinite(k)) continue;
      abs_sum += std::abs(k);
      signed_sum += k;
      ++count;
    }
    if (count == 0 || abs_sum <= 1e-9) return info;
    info.avg_abs = abs_sum / static_cast<double>(count);
    info.signed_mean = signed_sum / static_cast<double>(count);
    info.sign_consistency = std::min(1.0, std::abs(signed_sum) / abs_sum);
    return info;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}
