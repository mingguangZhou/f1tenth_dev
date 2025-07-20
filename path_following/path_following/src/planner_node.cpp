#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

// Simple 2D point.
struct Vec2f {
  double x;
  double y;
};

// PlannerNode subscribes to left and right boundary paths, computes a smoothed
// centerline starting at least kMinStartX ahead of the vehicle, and publishes
// the trajectory. It also publishes correspondence markers.
class PlannerNode : public rclcpp::Node {
 public:
  PlannerNode()
      : Node("planner_node"),
        resample_count_(100),   // Number of points for resampling
        smooth_window_(3),      // Half-window size for smoothing
        min_start_x_(1.0) {     // Minimum X for first point
    using std::placeholders::_1;

    left_subscriber_ = create_subscription<nav_msgs::msg::Path>(
        "left_boundary", rclcpp::QoS(10),
        std::bind(&PlannerNode::OnLeftBoundary, this, _1));

    right_subscriber_ = create_subscription<nav_msgs::msg::Path>(
        "right_boundary", rclcpp::QoS(10),
        std::bind(&PlannerNode::OnRightBoundary, this, _1));

    trajectory_publisher_ = create_publisher<nav_msgs::msg::Path>(
        "trajectory", rclcpp::QoS(10));

    correspondence_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "correspondence_markers", rclcpp::QoS(10));

    RCLCPP_INFO(get_logger(), "PlannerNode initialized.");
  }

 private:
  // Subscribers and publishers.
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr left_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr right_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      correspondence_publisher_;

  // Latest boundary messages.
  nav_msgs::msg::Path::SharedPtr left_path_;
  nav_msgs::msg::Path::SharedPtr right_path_;

  // Resampling and smoothing parameters.
  const size_t resample_count_;
  const int smooth_window_;
  const double min_start_x_;

  // Callback invoked when a new left boundary arrives.
  void OnLeftBoundary(const nav_msgs::msg::Path::SharedPtr msg) {
    left_path_ = msg;
    TryPublishTrajectory();
  }

  // Callback invoked when a new right boundary arrives.
  void OnRightBoundary(const nav_msgs::msg::Path::SharedPtr msg) {
    right_path_ = msg;
    TryPublishTrajectory();
  }

  // Attempts to compute and publish the centerline trajectory.
  void TryPublishTrajectory() {
    if (!left_path_ || !right_path_) {
      return;
    }

    auto left_pts = ConvertToVec(left_path_);
    auto right_pts = ConvertToVec(right_path_);
    if (left_pts.size() < 2 || right_pts.size() < 2) {
      RCLCPP_WARN(get_logger(), "Insufficient boundary points.");
      return;
    }

    auto left_rs = Resample(left_pts, resample_count_);
    auto right_rs = Resample(right_pts, resample_count_);

    // Build two candidate trajectories and choose the forward one.
    auto traj = BuildCenterline(left_rs, right_rs);

    if (!traj.poses.empty()) {
      trajectory_publisher_->publish(traj);
    }
  }

  // Converts a ROS Path to a vector of Vec2f.
  static std::vector<Vec2f> ConvertToVec(
      const nav_msgs::msg::Path::SharedPtr &path) {
    std::vector<Vec2f> pts;
    pts.reserve(path->poses.size());
    for (const auto &ps : path->poses) {
      pts.push_back({ps.pose.position.x, ps.pose.position.y});
    }
    return pts;
  }

  // Reverses the order of points in a vector.
  static std::vector<Vec2f> Reverse(const std::vector<Vec2f> &v) {
    std::vector<Vec2f> out(v);
    std::reverse(out.begin(), out.end());
    return out;
  }

  // Returns true if the first segment of the path goes in +X direction.
  static bool IsForward(const nav_msgs::msg::Path &path) {
    if (path.poses.size() < 2) {
      return false;
    }
    double dx = path.poses[1].pose.position.x - path.poses[0].pose.position.x;
    return dx >= 0.0;
  }

  // Builds a smooth centerline with the first point at least min_start_x_ ahead.
  nav_msgs::msg::Path BuildCenterline(
      const std::vector<Vec2f> &A, const std::vector<Vec2f> &B) {
    auto pairs = MatchBoundaries(A, B);
    std::vector<Vec2f> center;
    center.reserve(pairs.size());

    for (const auto &pr : pairs) {
      const auto &a = A[pr.first];
      const auto &b = B[pr.second];
      center.push_back({0.5 * (a.x + b.x), 0.5 * (a.y + b.y)});
    }

    center = Smooth(center, smooth_window_);

    // Ensure the first point is at least min_start_x_ ahead.
    size_t idx = 0;
    while (idx < center.size() && center[idx].x < min_start_x_) {
      ++idx;
    }
    if (idx > 0) {
      center.erase(center.begin(), center.begin() + idx);
    }

    nav_msgs::msg::Path out;
    out.header = left_path_->header;
    out.header.stamp = now();
    out.poses.reserve(center.size());
    for (const auto &p : center) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = out.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.orientation.w = 1.0;
      out.poses.push_back(ps);
    }
    return out;
  }

  // Computes cumulative distances along a polyline.
  static std::vector<double> CumDist(const std::vector<Vec2f> &v) {
    std::vector<double> d(v.size(), 0.0);
    for (size_t i = 1; i < v.size(); ++i) {
      double dx = v[i].x - v[i - 1].x;
      double dy = v[i].y - v[i - 1].y;
      d[i] = d[i - 1] + std::hypot(dx, dy);
    }
    return d;
  }

  // Resamples a polyline to N points uniformly along its length.
  static std::vector<Vec2f> Resample(
      const std::vector<Vec2f> &v, size_t N) {
    auto d = CumDist(v);
    double length = d.back();
    std::vector<Vec2f> out;
    out.reserve(N);

    for (size_t i = 0; i < N; ++i) {
      double t = length * static_cast<double>(i) / (N - 1);
      auto it = std::lower_bound(d.begin(), d.end(), t);
      size_t j = std::distance(d.begin(), it);

      if (j == 0) {
        out.push_back(v.front());
      } else if (j >= v.size()) {
        out.push_back(v.back());
      } else {
        double alpha = (t - d[j - 1]) / (d[j] - d[j - 1]);
        Vec2f p;
        p.x = v[j - 1].x + alpha * (v[j].x - v[j - 1].x);
        p.y = v[j - 1].y + alpha * (v[j].y - v[j - 1].y);
        out.push_back(p);
      }
    }
    return out;
  }

  // Matches two polylines via greedy nearest-neighbor seeding and lock-step march.
  static std::vector<std::pair<size_t, size_t>> MatchBoundaries(
      const std::vector<Vec2f> &L, const std::vector<Vec2f> &R) {
    std::vector<std::pair<size_t, size_t>> matches;
    if (L.empty() || R.empty()) {
      return matches;
    }

    auto NearestIndex = [&](const Vec2f &p,
                            const std::vector<Vec2f> &pts) {
      size_t best_idx = 0;
      double best_dist2 = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < pts.size(); ++i) {
        double dx = pts[i].x - p.x;
        double dy = pts[i].y - p.y;
        double dist2 = dx * dx + dy * dy;
        if (dist2 < best_dist2) {
          best_dist2 = dist2;
          best_idx = i;
        }
      }
      return std::make_pair(best_idx, best_dist2);
    };

    auto [r0, distL] = NearestIndex(L[0], R);
    auto [l0, distR] = NearestIndex(R[0], L);

    size_t i = 0;
    size_t j = (distL < distR ? r0 : 0);
    if (distR <= distL) {
      i = l0;
      j = 0;
    }

    while (i < L.size() && j < R.size()) {
      matches.emplace_back(i, j);
      ++i;
      ++j;
    }
    return matches;
  }

  // Applies a simple moving-average smoother to a polyline.
  static std::vector<Vec2f> Smooth(const std::vector<Vec2f> &in,
                                    int window) {
    size_t M = in.size();
    std::vector<Vec2f> out(in);
    for (size_t i = 0; i < M; ++i) {
      double sum_x = 0.0;
      double sum_y = 0.0;
      int count = 0;
      for (int w = -window; w <= window; ++w) {
        int idx = static_cast<int>(i) + w;
        if (idx >= 0 && idx < static_cast<int>(M)) {
          sum_x += in[idx].x;
          sum_y += in[idx].y;
          ++count;
        }
      }
      out[i].x = sum_x / count;
      out[i].y = sum_y / count;
    }
    return out;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
