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

// Generates optimized racing line from left and right boundaries
class PlannerNode : public rclcpp::Node {
 public:
  PlannerNode()
      : Node("planner_node"),
        resample_count_(100) {         
    using std::placeholders::_1;

    declare_parameter<double>("min_start_x", 1.0);
    declare_parameter<int>("racing_line_window_size", 5);
    declare_parameter<double>("min_weight", 0.1);
    declare_parameter<double>("max_weight", 0.9);
    declare_parameter<int>("max_optimization_iterations", 3);
    declare_parameter<double>("weight_step", 0.05);
    declare_parameter<double>("max_lateral_offset", 0.5);
   
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

    min_start_x_ = get_parameter("min_start_x").as_double();
    

    racing_line_window_size_ = get_parameter("racing_line_window_size").as_int();
    min_weight_ = get_parameter("min_weight").as_double();
    max_weight_ = get_parameter("max_weight").as_double();
    max_optimization_iterations_ = get_parameter("max_optimization_iterations").as_int();
    max_lateral_offset_ = get_parameter("max_lateral_offset").as_double();

    RCLCPP_INFO(get_logger(), "PlannerNode initialized.");
  }

 private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr left_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr right_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      correspondence_publisher_;

  nav_msgs::msg::Path::SharedPtr left_path_;
  nav_msgs::msg::Path::SharedPtr right_path_;

  const size_t resample_count_;
  double min_start_x_;
  
  int racing_line_window_size_;
  double min_weight_;
  double max_weight_;
  int max_optimization_iterations_;
  double max_lateral_offset_;

  void OnLeftBoundary(const nav_msgs::msg::Path::SharedPtr msg) {
    left_path_ = msg;
    TryPublishTrajectory();
  }

  void OnRightBoundary(const nav_msgs::msg::Path::SharedPtr msg) {
    right_path_ = msg;
    TryPublishTrajectory();
  }

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


    auto traj = BuildCenterline(left_rs, right_rs);

    if (!traj.poses.empty()) {
      trajectory_publisher_->publish(traj);
    }
  }

  static std::vector<Vec2f> ConvertToVec(
      const nav_msgs::msg::Path::SharedPtr &path) {
    std::vector<Vec2f> pts;
    pts.reserve(path->poses.size());
    for (const auto &ps : path->poses) {
      pts.push_back({ps.pose.position.x, ps.pose.position.y});
    }
    return pts;
  }

  static std::vector<Vec2f> Reverse(const std::vector<Vec2f> &v) {
    std::vector<Vec2f> out(v);
    std::reverse(out.begin(), out.end());
    return out;
  }

  static bool IsForward(const nav_msgs::msg::Path &path) {
    if (path.poses.size() < 2) {
      return false;
    }
    double dx = path.poses[1].pose.position.x - path.poses[0].pose.position.x;
    return dx >= 0.0;
  }

  nav_msgs::msg::Path BuildCenterline(
      const std::vector<Vec2f> &A, const std::vector<Vec2f> &B) {
    auto pairs = MatchBoundaries(A, B);
    std::vector<Vec2f> center;
    std::vector<double> weights;
    center.reserve(pairs.size());
    weights.reserve(pairs.size());


    for (const auto &pr : pairs) {
      const auto &a = A[pr.first];
      const auto &b = B[pr.second];
      center.push_back({0.5 * (a.x + b.x), 0.5 * (a.y + b.y)});
      weights.push_back(0.5);
    }


    weights = OptimizeWeightsForRacingLine(A, B, pairs);


    center.clear();
    for (size_t i = 0; i < pairs.size(); ++i) {
      const auto &pr = pairs[i];
      const auto &a = A[pr.first];
      const auto &b = B[pr.second];
      double w = weights[i];
      center.push_back({w * a.x + (1.0 - w) * b.x, w * a.y + (1.0 - w) * b.y});
    }

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

  std::vector<double> OptimizeWeightsForRacingLine(
      const std::vector<Vec2f> &A, const std::vector<Vec2f> &B,
      const std::vector<std::pair<size_t, size_t>> &pairs) {
    
    // First, create initial centerline with 0.5 weights
    std::vector<Vec2f> centerline;
    centerline.reserve(pairs.size());
    for (const auto &pr : pairs) {
      const auto &a = A[pr.first];
      const auto &b = B[pr.second];
      centerline.push_back({0.5 * (a.x + b.x), 0.5 * (a.y + b.y)});
    }
    

    SmoothCurveFitting(centerline, max_optimization_iterations_, racing_line_window_size_, max_lateral_offset_);
    

    std::vector<double> weights;
    weights.reserve(pairs.size());
    
    for (size_t i = 0; i < pairs.size(); ++i) {
      const auto &pr = pairs[i];
      const auto &a = A[pr.first];
      const auto &b = B[pr.second];
      const auto &p = centerline[i];
      
      // Solve for w using least squares approach
      double numerator = (p.x - b.x) * (a.x - b.x) + (p.y - b.y) * (a.y - b.y);
      double denominator = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
      
      double w = 0.5;
      if (denominator > 1e-6) {
        w = numerator / denominator;
      }
      

      w = std::max(min_weight_, std::min(max_weight_, w));
      weights.push_back(w);
    }
    

    weights = SmoothWeights(weights, 2);
    
    return weights;
  }


  void SmoothCurveFitting(std::vector<Vec2f> &points, int num_iterations, 
                          int window_size, double max_lateral_offset) {
    int num_points = points.size();
    if (num_points <= 2 || window_size <= 2) return;
    
    window_size = std::min(window_size, num_points);
    
    std::vector<Vec2f> moving_vectors(num_points, {0.0, 0.0});
    std::vector<Vec2f> accumulated_average_curvature(num_points, {0.0, 0.0});
    std::vector<Vec2f> diff(num_points - 1, {0.0, 0.0});
    std::vector<Vec2f> total_moving_vectors(num_points, {0.0, 0.0});
    std::vector<Vec2f> curvature_difference(num_points, {0.0, 0.0});
    std::vector<Vec2f> half_step_moved_points(num_points, {0.0, 0.0});
    std::vector<Vec2f> one_step_moved_points(num_points, {0.0, 0.0});
    std::vector<int> accumulated_num_of_windows(num_points, 0);
    std::vector<double> inv_num_of_windows(num_points);
    
    const double max_lateral_offset_sqr = max_lateral_offset * max_lateral_offset;
    const double kEpsilon = 1e-6;
    

    for (int first = 0; first <= num_points - window_size; first++) {
      int last = first + window_size - 1;
      accumulated_num_of_windows[first + 1] += 1;
      accumulated_num_of_windows[last] -= 1;
    }
    
    int num_of_windows = 0;
    for (int i = 0; i < num_points; ++i) {
      num_of_windows += accumulated_num_of_windows[i];
      inv_num_of_windows[i] = 1.0 / std::max<double>(kEpsilon, num_of_windows);
    }
    
    while (num_iterations--) {
      CalculateCurvatureDifference(points, inv_num_of_windows, window_size,
                                   &accumulated_average_curvature, &diff, &curvature_difference);
      
      double cost_with_zero_step = 0.0;
      for (const auto &difference : curvature_difference) {
        cost_with_zero_step += difference.x * difference.x + difference.y * difference.y;
      }
      

      for (int i = 1; i + 1 < num_points; ++i) {
        Vec2f direction = {diff[i - 1].x + diff[i].x, diff[i - 1].y + diff[i].y};
        double length = std::hypot(direction.x, direction.y);
        if (length > kEpsilon) {
          direction.x /= length;
          direction.y /= length;

          double temp = direction.x;
          direction.x = -direction.y;
          direction.y = temp;
          

          double projection = direction.x * curvature_difference[i].x + 
                             direction.y * curvature_difference[i].y;
          moving_vectors[i] = {projection * direction.x, projection * direction.y};
        }
      }
      

      for (int i = 0; i < num_points; ++i) {
        half_step_moved_points[i] = {points[i].x + 0.5 * moving_vectors[i].x,
                                     points[i].y + 0.5 * moving_vectors[i].y};
        one_step_moved_points[i] = {points[i].x + moving_vectors[i].x,
                                    points[i].y + moving_vectors[i].y};
      }
      
      CalculateCurvatureDifference(half_step_moved_points, inv_num_of_windows, window_size,
                                   &accumulated_average_curvature, &diff, &curvature_difference);
      double cost_with_half_step = 0.0;
      for (const auto &difference : curvature_difference) {
        cost_with_half_step += difference.x * difference.x + difference.y * difference.y;
      }
      
      CalculateCurvatureDifference(one_step_moved_points, inv_num_of_windows, window_size,
                                   &accumulated_average_curvature, &diff, &curvature_difference);
      double cost_with_one_step = 0.0;
      for (const auto &difference : curvature_difference) {
        cost_with_one_step += difference.x * difference.x + difference.y * difference.y;
      }
      

      double quadratic_a = 2 * cost_with_zero_step - 4 * cost_with_half_step + 2 * cost_with_one_step;
      double quadratic_b = -3 * cost_with_zero_step + 4 * cost_with_half_step - cost_with_one_step;
      double quadratic_c = cost_with_zero_step;
      

      constexpr double kStepSizeLowerBound = 0.0;
      constexpr double kStepSizeUpperBound = 1.0;
      double step_size = kStepSizeLowerBound;
      
      if (quadratic_a > 0) {
        double symmetry_axis = -quadratic_b / (2 * quadratic_a);
        step_size = std::max(kStepSizeLowerBound, std::min(kStepSizeUpperBound, symmetry_axis));
      } else if (quadratic_a == 0) {
        if (quadratic_b < 0) {
          step_size = kStepSizeUpperBound;
        }
      }
      
      if (step_size < kEpsilon) break;
      
      double expected_cost = quadratic_a * step_size * step_size + 
                            quadratic_b * step_size + quadratic_c;
      if (cost_with_zero_step - expected_cost < kEpsilon) break;
      

      bool cross_max_offset = false;
      for (int i = 1; i + 1 < num_points; ++i) {
        moving_vectors[i].x *= step_size;
        moving_vectors[i].y *= step_size;
        total_moving_vectors[i].x += moving_vectors[i].x;
        total_moving_vectors[i].y += moving_vectors[i].y;
        
        double total_offset_sqr = total_moving_vectors[i].x * total_moving_vectors[i].x +
                                 total_moving_vectors[i].y * total_moving_vectors[i].y;
        if (total_offset_sqr > max_lateral_offset_sqr) {
          cross_max_offset = true;
          break;
        }
      }
      
      if (cross_max_offset) break;
      

      for (int i = 0; i < num_points; i++) {
        points[i].x += moving_vectors[i].x;
        points[i].y += moving_vectors[i].y;
      }
    }
  }
  

  void CalculateCurvatureDifference(const std::vector<Vec2f> &points,
                                    const std::vector<double> &inv_num_of_windows,
                                    int window_size,
                                    std::vector<Vec2f> *accumulated_average_curvature,
                                    std::vector<Vec2f> *diff,
                                    std::vector<Vec2f> *curvature_difference) {
    int num_points = points.size();
    if (num_points <= 2 || window_size <= 2) return;
    

    for (auto &curv : *accumulated_average_curvature) {
      curv = {0.0, 0.0};
    }
    

    for (int i = 0; i + 1 < num_points; i++) {
      (*diff)[i] = {points[i + 1].x - points[i].x, points[i + 1].y - points[i].y};
    }
    
    double inv_window_size_minus_2 = 1.0 / (window_size - 2);
    for (int first = 0; first <= num_points - window_size; first++) {
      int last = first + window_size - 1;
      Vec2f average_curvature = {
        ((*diff)[last - 1].x - (*diff)[first].x) * inv_window_size_minus_2,
        ((*diff)[last - 1].y - (*diff)[first].y) * inv_window_size_minus_2
      };
      (*accumulated_average_curvature)[first + 1].x += average_curvature.x;
      (*accumulated_average_curvature)[first + 1].y += average_curvature.y;
      (*accumulated_average_curvature)[last].x -= average_curvature.x;
      (*accumulated_average_curvature)[last].y -= average_curvature.y;
    }
    
    Vec2f sum_of_average_curvature_for_windows = {0.0, 0.0};
    for (auto &curv_diff : *curvature_difference) {
      curv_diff = {0.0, 0.0};
    }
    
    for (int i = 1; i + 1 < num_points; ++i) {
      const Vec2f curvature = {
        (*diff)[i].x - (*diff)[i - 1].x,
        (*diff)[i].y - (*diff)[i - 1].y
      };
      sum_of_average_curvature_for_windows.x += (*accumulated_average_curvature)[i].x;
      sum_of_average_curvature_for_windows.y += (*accumulated_average_curvature)[i].y;
      
      (*curvature_difference)[i] = {
        curvature.x - sum_of_average_curvature_for_windows.x * inv_num_of_windows[i],
        curvature.y - sum_of_average_curvature_for_windows.y * inv_num_of_windows[i]
      };
    }
  }


  static std::vector<double> CumDist(const std::vector<Vec2f> &v) {
    std::vector<double> d(v.size(), 0.0);
    for (size_t i = 1; i < v.size(); ++i) {
      double dx = v[i].x - v[i - 1].x;
      double dy = v[i].y - v[i - 1].y;
      d[i] = d[i - 1] + std::hypot(dx, dy);
    }
    return d;
  }


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


  static std::vector<double> SmoothWeights(const std::vector<double> &in,
                                           int window) {
    size_t M = in.size();
    std::vector<double> out(in);
    for (size_t i = 0; i < M; ++i) {
      double sum = 0.0;
      int count = 0;
      for (int w = -window; w <= window; ++w) {
        int idx = static_cast<int>(i) + w;
        if (idx >= 0 && idx < static_cast<int>(M)) {
          sum += in[idx];
          ++count;
        }
      }
      out[i] = sum / count;
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
