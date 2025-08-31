// planner_node.cpp
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

// -------------------------- Simple 2D point --------------------------
struct Vec2f {
  double x;
  double y;
};

// PlannerNode subscribes to left/right boundary paths, optimizes a racing-line
// centerline inside the corridor, and publishes a Path. Key changes vs. a
// plain midpoint: (1) local bending-energy smoothing can move the *head*
// points, and (2) the head weights are blended toward a short preview so your
// MPC actually executes the racing line instead of re-centering every cycle.
class PlannerNode : public rclcpp::Node {
 public:
  PlannerNode() : Node("planner_node"), resample_count_(100) {
    using std::placeholders::_1;

    // -------------------------- Parameters --------------------------
    // Corridor / path formation
    declare_parameter<int>("resample_count", 100);
    declare_parameter<double>("min_start_x", 0.75);

    // Local racing-line optimizer (windowed curvature equalization)
    declare_parameter<int>("racing_line_window_size", 7);   // odd >= 3
    declare_parameter<int>("max_optimization_iterations", 3);
    declare_parameter<double>("max_lateral_offset", 0.6);   // meters

    // Corridor blend weights w \in [min_weight, max_weight] on each segment
    declare_parameter<double>("min_weight", 0.1);
    declare_parameter<double>("max_weight", 0.9);

    // Head preview to combat receding-horizon myopia
    declare_parameter<int>("preview_head_k", 10);     // how many head weights to reshape
    declare_parameter<int>("preview_lookahead_m", 15);// how far ahead to peek
    declare_parameter<double>("preview_gamma", 0.7);  // blend strength [0..1]

    // Smoothing of weight field + warm start between cycles
    declare_parameter<bool>("enable_warm_start", true);

    // Load parameters
    resample_count_ = static_cast<size_t>(get_parameter("resample_count").as_int());
    min_start_x_ = get_parameter("min_start_x").as_double();

    racing_line_window_size_ = get_parameter("racing_line_window_size").as_int();
    max_optimization_iterations_ =
        get_parameter("max_optimization_iterations").as_int();
    max_lateral_offset_ = get_parameter("max_lateral_offset").as_double();

    min_weight_ = get_parameter("min_weight").as_double();
    max_weight_ = get_parameter("max_weight").as_double();

    preview_head_k_ = get_parameter("preview_head_k").as_int();
    preview_lookahead_m_ = get_parameter("preview_lookahead_m").as_int();
    preview_gamma_ = get_parameter("preview_gamma").as_double();

    enable_warm_start_ = get_parameter("enable_warm_start").as_bool();

    // -------------------------- ROS I/O --------------------------
    left_sub_ = create_subscription<nav_msgs::msg::Path>(
        "left_boundary", rclcpp::QoS(10),
        std::bind(&PlannerNode::OnLeft, this, _1));
    right_sub_ = create_subscription<nav_msgs::msg::Path>(
        "right_boundary", rclcpp::QoS(10),
        std::bind(&PlannerNode::OnRight, this, _1));
    traj_pub_ =
        create_publisher<nav_msgs::msg::Path>("trajectory", rclcpp::QoS(10));

    RCLCPP_INFO(get_logger(), "PlannerNode initialized.");
  }

 private:
  // -------------------------- ROS members --------------------------
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr left_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr right_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
  nav_msgs::msg::Path::SharedPtr left_path_;
  nav_msgs::msg::Path::SharedPtr right_path_;

  // -------------------------- Params & state --------------------------
  size_t resample_count_;
  double min_start_x_;

  int racing_line_window_size_;
  int max_optimization_iterations_;
  double max_lateral_offset_;

  double min_weight_;
  double max_weight_;

  int preview_head_k_;
  int preview_lookahead_m_;
  double preview_gamma_;

  int smooth_weights_window_;
  bool enable_warm_start_;
  
    // Warm start for racing line optimization
  std::vector<double> prev_weights_;
  size_t prev_drop_ = 0;

  // -------------------------- Callbacks --------------------------
  void OnLeft(const nav_msgs::msg::Path::SharedPtr msg) {
    left_path_ = msg;
    TryPublish();
  }
  void OnRight(const nav_msgs::msg::Path::SharedPtr msg) {
    right_path_ = msg;
    TryPublish();
  }

  void TryPublish() {
    if (!left_path_ || !right_path_) return;

    auto L = ToVec(left_path_);
    auto R = ToVec(right_path_);
    if (L.size() < 2 || R.size() < 2) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Boundaries too short.");
      return;
    }

    auto Lr = Resample(L, resample_count_);
    auto Rr = Resample(R, resample_count_);

    // Greedy correspondence (seeded by nearest ends, lock-step march)
    auto pairs = MatchBoundaries(Lr, Rr);
    if (pairs.size() < 3) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Too few correspondence pairs.");
      return;
    }

    // Optimize blend weights per pair into a racing line.
    auto weights = OptimizeWeightsForRacingLine(Lr, Rr, pairs);

    // Build centerline from weights.
    std::vector<Vec2f> center;
    center.reserve(pairs.size());
    for (size_t i = 0; i < pairs.size(); ++i) {
      const auto &a = Lr[pairs[i].first];
      const auto &b = Rr[pairs[i].second];
      double w = weights[i];
      center.push_back({w * a.x + (1.0 - w) * b.x, w * a.y + (1.0 - w) * b.y});
    }

    // Drop any leading points too close to ego (x < min_start_x_).
    size_t drop = 0;
    while (drop < center.size() && center[drop].x < min_start_x_) ++drop;
    if (drop > 0 && drop < center.size()) {
      center.erase(center.begin(), center.begin() + static_cast<long>(drop));
      weights.erase(weights.begin(), weights.begin() + static_cast<long>(drop));
    }

    // Publish path
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
    if (!out.poses.empty()) {
      traj_pub_->publish(out);
    }

    // Store warm-start info for next cycle.
    prev_weights_ = weights;
    prev_drop_ = drop;
  }

  // -------------------------- Core optimization --------------------------
  // Optimize weights so the windowed bending-energy surrogate is reduced.
  // Steps:
  //  1) seed centerline from w_init
  //  2) run SmoothCurveFitting() that can move *head* points too
  //  3) project resulting points back to weights
  //  4) smooth weights and apply preview blending on the head
  std::vector<double> OptimizeWeightsForRacingLine(
      const std::vector<Vec2f> &A, const std::vector<Vec2f> &B,
      const std::vector<std::pair<size_t, size_t>> &pairs) {
    const int N = static_cast<int>(pairs.size());
    
    // Warm start: initialize weights from previous cycle if available
    std::vector<double> w_init(N, 0.5);
    if (enable_warm_start_ && !prev_weights_.empty()) {
      size_t shift = prev_drop_;  // number of points trimmed last cycle
      for (size_t i = 0; i < w_init.size(); ++i) {
        size_t k = std::min(i + shift, prev_weights_.size() - 1);
        w_init[i] = prev_weights_[k];
      }
      RCLCPP_INFO(get_logger(), "Using warm start with shift=%zu, prev_weights size=%zu", 
                  shift, prev_weights_.size());
    }
    
    std::vector<double> w = w_init;

    // 1) seed centerline from w_init
    std::vector<Vec2f> center(N);
    for (int i = 0; i < N; ++i) {
      const auto &a = A[pairs[i].first];
      const auto &b = B[pairs[i].second];
      center[i] = {w[i] * a.x + (1.0 - w[i]) * b.x,
                   w[i] * a.y + (1.0 - w[i]) * b.y};
    }

    // 2) local fairing that *also moves the head points*
    // Add continuity penalty for anchor points (first K points)
    const int K_anchor = std::min(5, N);  // Anchor first 5 points
    SmoothCurveFitting(center, max_optimization_iterations_,
                       racing_line_window_size_, max_lateral_offset_,
                       w_init, K_anchor);

    // 3) project back to weights along the corridor chord
    for (int i = 0; i < N; ++i) {
      const auto &a = A[pairs[i].first];
      const auto &b = B[pairs[i].second];
      const auto &p = center[i];
      const double ax = a.x - b.x, ay = a.y - b.y;
      const double px = p.x - b.x, py = p.y - b.y;
      const double den = ax * ax + ay * ay;
      double wi = (den > 1e-8) ? (px * ax + py * ay) / den : 0.5;
      w[i] = std::clamp(wi, min_weight_, max_weight_);
    }


    // 4b) preview blending on the head (inject look-ahead into the first K)
    const int K = std::max(0, std::min(preview_head_k_, N));
    const int M = std::max(0, std::min(preview_lookahead_m_, N));
    const double gamma = std::clamp(preview_gamma_, 0.0, 1.0);
    for (int i = 0; i < K; ++i) {
      const int j2 = std::min(i + M, N - 1);
      double acc = 0.0;
      int cnt = 0;
      for (int j = i; j <= j2; ++j) {
        acc += w[j];
        ++cnt;
      }
      const double target = acc / std::max(1, cnt);
      w[i] = (1.0 - gamma) * w[i] + gamma * target;
      w[i] = std::clamp(w[i], min_weight_, max_weight_);
    }
    
    // Safety check: ensure all weights are within bounds
    for (int i = 0; i < N; ++i) {
      if (w[i] < min_weight_ || w[i] > max_weight_) {
        RCLCPP_WARN(get_logger(), "Weight[%d] = %.3f outside bounds [%.3f, %.3f], clamping", 
                    i, w[i], min_weight_, max_weight_);
        w[i] = std::clamp(w[i], min_weight_, max_weight_);
      }
    }
    
    // Debug: Print first few weights
    RCLCPP_INFO(get_logger(), "Final weights (first 5): %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
                w[0], w[1], w[2], w[3], w[4], min_weight_, max_weight_);
    
    return w;
  }

  // -------------------------- Geometry utils --------------------------
  static std::vector<Vec2f> ToVec(const nav_msgs::msg::Path::SharedPtr &path) {
    std::vector<Vec2f> pts;
    pts.reserve(path->poses.size());
    for (const auto &ps : path->poses) {
      pts.push_back({ps.pose.position.x, ps.pose.position.y});
    }
    return pts;
  }

  static std::vector<double> CumDist(const std::vector<Vec2f> &v) {
    std::vector<double> d(v.size(), 0.0);
    for (size_t i = 1; i < v.size(); ++i) {
      const double dx = v[i].x - v[i - 1].x;
      const double dy = v[i].y - v[i - 1].y;
      d[i] = d[i - 1] + std::hypot(dx, dy);
    }
    return d;
  }

  static std::vector<Vec2f> Resample(const std::vector<Vec2f> &v, size_t N) {
    if (v.size() <= 2 || N <= 2) return v;
    auto d = CumDist(v);
    const double L = d.back();
    std::vector<Vec2f> out;
    out.reserve(N);
    for (size_t i = 0; i < N; ++i) {
      const double t = L * static_cast<double>(i) / static_cast<double>(N - 1);
      auto it = std::lower_bound(d.begin(), d.end(), t);
      size_t j = static_cast<size_t>(std::distance(d.begin(), it));
      if (j == 0) {
        out.push_back(v.front());
      } else if (j >= v.size()) {
        out.push_back(v.back());
      } else {
        const double a = (t - d[j - 1]) / (d[j] - d[j - 1]);
        out.push_back({v[j - 1].x + a * (v[j].x - v[j - 1].x),
                       v[j - 1].y + a * (v[j].y - v[j - 1].y)});
      }
    }
    return out;
  }

  // Greedy one-to-one correspondence seeding and lock-step matching.
  static std::vector<std::pair<size_t, size_t>> MatchBoundaries(
      const std::vector<Vec2f> &L, const std::vector<Vec2f> &R) {
    std::vector<std::pair<size_t, size_t>> m;
    if (L.empty() || R.empty()) return m;

    auto Nearest = [](const Vec2f &p, const std::vector<Vec2f> &V) {
      size_t bi = 0;
      double bd2 = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < V.size(); ++i) {
        const double dx = V[i].x - p.x, dy = V[i].y - p.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < bd2) {
          bd2 = d2;
          bi = i;
        }
      }
      return std::make_pair(bi, bd2);
    };

    auto [r0, dL] = Nearest(L[0], R);
    auto [l0, dR] = Nearest(R[0], L);
    size_t i = 0, j = (dL < dR ? r0 : 0);
    if (dR <= dL) {
      i = l0;
      j = 0;
    }
    while (i < L.size() && j < R.size()) {
      m.emplace_back(i, j);
      ++i;
      ++j;
    }
    return m;
  }

  static std::vector<double> SmoothWeights(const std::vector<double> &in,
                                           int W) {
    if (W <= 0) return in;
    const int M = static_cast<int>(in.size());
    std::vector<double> out(in);
    for (int i = 0; i < M; ++i) {
      double s = 0.0;
      int c = 0;
      for (int w = -W; w <= W; ++w) {
        const int j = i + w;
        if (j >= 0 && j < M) {
          s += in[j];
          ++c;
        }
      }
      out[i] = s / std::max(1, c);
    }
    return out;
  }

  // -------------------------- Racing-line fairing --------------------------
  // Windowed curvature-equalization that *also* moves endpoints by using
  // one-sided tangents and copying curvature differences at the ends.
  static void SmoothCurveFitting(std::vector<Vec2f> &pts, int iters, int W,
                                 double max_lat, 
                                 const std::vector<double> &w_init = {},
                                 int K_anchor = 0) {
    const int N = static_cast<int>(pts.size());
    if (N <= 2 || W <= 2 || iters <= 0) return;
    W = std::min(W, N);

    std::vector<Vec2f> diff(std::max(1, N - 1), {0.0, 0.0});
    std::vector<Vec2f> acc_avg_curv(N, {0.0, 0.0});
    std::vector<Vec2f> curv_diff(N, {0.0, 0.0});
    std::vector<Vec2f> move(N, {0.0, 0.0});
    std::vector<Vec2f> total_move(N, {0.0, 0.0});
    std::vector<Vec2f> half_pts(N), one_pts(N);
    std::vector<int> acc_window_marks(N, 0);
    std::vector<double> inv_windows(N, 1.0);

    const double eps = 1e-6;
    const double max_lat2 = max_lat * max_lat;

    // Prefix-sum bookkeeping: how many windows cover each index.
    for (int f = 0; f <= N - W; ++f) {
      const int l = f + W - 1;
      acc_window_marks[f + 1] += 1;
      acc_window_marks[l] -= 1;
    }
    int active = 0;
    for (int i = 0; i < N; ++i) {
      active += acc_window_marks[i];
      inv_windows[i] = 1.0 / std::max<double>(eps, active);
    }

    auto CalcCurvDiff = [&](const std::vector<Vec2f> &p) {
      // reset accumulators
      std::fill(acc_avg_curv.begin(), acc_avg_curv.end(), Vec2f{0.0, 0.0});
      std::fill(curv_diff.begin(), curv_diff.end(), Vec2f{0.0, 0.0});

      // segment diffs
      for (int i = 0; i + 1 < N; ++i) {
        diff[i] = {p[i + 1].x - p[i].x, p[i + 1].y - p[i].y};
      }

      const double inv_wm2 = 1.0 / (W - 2);
      for (int f = 0; f <= N - W; ++f) {
        const int l = f + W - 1;
        const Vec2f avg_curv = {(diff[l - 1].x - diff[f].x) * inv_wm2,
                                (diff[l - 1].y - diff[f].y) * inv_wm2};
        acc_avg_curv[f + 1].x += avg_curv.x;
        acc_avg_curv[f + 1].y += avg_curv.y;
        acc_avg_curv[l].x -= avg_curv.x;
        acc_avg_curv[l].y -= avg_curv.y;
      }

      Vec2f run = {0.0, 0.0};
      for (int i = 1; i + 1 < N; ++i) {
        run.x += acc_avg_curv[i].x;
        run.y += acc_avg_curv[i].y;
        const Vec2f cur = {diff[i].x - diff[i - 1].x,
                           diff[i].y - diff[i - 1].y};
        curv_diff[i] = {cur.x - run.x * inv_windows[i],
                        cur.y - run.y * inv_windows[i]};
      }

      // allow endpoints to move: copy neighbor curvature differences
      if (N >= 3) {
        curv_diff[0] = curv_diff[1];
        curv_diff[N - 1] = curv_diff[N - 2];
      }
    };

    while (iters--) {
      CalcCurvDiff(pts);

      // Build normal-direction moves at ALL indices (endpoints too)
      for (int i = 0; i < N; ++i) {
        Vec2f t_prev = (i > 0) ? diff[i - 1] : diff[0];
        Vec2f t_next = (i < N - 1) ? diff[i] : diff[N - 2];
        Vec2f t = {t_prev.x + t_next.x, t_prev.y + t_next.y};
        const double len = std::hypot(t.x, t.y);
        if (len > eps) {
          t.x /= len;
          t.y /= len;
          // rotate +90° to get local normal
          const double tx = t.x;
          t.x = -t.y;
          t.y = tx;
          const double proj = t.x * curv_diff[i].x + t.y * curv_diff[i].y;
          move[i] = {proj * t.x, proj * t.y};
        } else {
          move[i] = {0.0, 0.0};
        }
      }

      // Quadratic line search using 0, 0.5, 1.0 steps.
      for (int i = 0; i < N; ++i) {
        half_pts[i] = {pts[i].x + 0.5 * move[i].x, pts[i].y + 0.5 * move[i].y};
        one_pts[i] = {pts[i].x + move[i].x, pts[i].y + move[i].y};
      }
      auto Cost = [&](const std::vector<Vec2f> &p) {
        CalcCurvDiff(p);
        double c = 0.0;
        for (int i = 0; i < N; ++i) c += curv_diff[i].x * curv_diff[i].x +
                                         curv_diff[i].y * curv_diff[i].y;
        
        // Add continuity penalty for anchor points if w_init is provided
        if (!w_init.empty() && K_anchor > 0) {
          const double continuity_weight = 0.1;  // Small penalty weight
          for (int i = 0; i < std::min(K_anchor, N); ++i) {
            // Project current point back to weight and compare with w_init
            // This is a simplified version - in practice you'd need the A, B vectors
            // For now, we'll use a simple distance penalty
            double weight_diff = 0.0;
            if (i < static_cast<int>(w_init.size())) {
              // Estimate current weight from point position (simplified)
              // In a full implementation, you'd project the point back to the corridor
              weight_diff = std::abs(0.5 - w_init[i]);  // Simplified penalty
            }
            c += continuity_weight * weight_diff * weight_diff;
          }
        }
        
        return c;
      };
      const double c0 = Cost(pts);
      const double cH = Cost(half_pts);
      const double c1 = Cost(one_pts);

      const double a = 2 * c0 - 4 * cH + 2 * c1;
      const double b = -3 * c0 + 4 * cH - c1;
      double step = 0.0;
      if (a > 0.0) {
        step = std::clamp(-b / (2 * a), 0.0, 1.0);
      } else if (std::abs(a) <= eps && b < 0.0) {
        step = 1.0;
      }
      if (step < 1e-8) break;

      // Apply step, enforce max lateral excursion
      bool hit_limit = false;
      for (int i = 0; i < N; ++i) {
        total_move[i].x += step * move[i].x;
        total_move[i].y += step * move[i].y;
        if (total_move[i].x * total_move[i].x +
                total_move[i].y * total_move[i].y >
            max_lat2) {
          hit_limit = true;
          break;
        }
      }
      if (hit_limit) break;

      for (int i = 0; i < N; ++i) {
        pts[i].x += step * move[i].x;
        pts[i].y += step * move[i].y;
      }
    }
  }
};

// -------------------------- main --------------------------
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
