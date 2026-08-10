#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;

class DriveArbitratorNode : public rclcpp::Node
{
public:
  DriveArbitratorNode()
  : Node("drive_arbitrator")
  {
    declareParameters();
    loadParameters();

    raceline_command_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      raceline_command_topic_, 10,
      std::bind(&DriveArbitratorNode::racelineCommandCallback, this, _1));
    reactive_command_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      reactive_command_topic_, 10,
      std::bind(&DriveArbitratorNode::reactiveCommandCallback, this, _1));
    path_status_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      path_status_topic_, 10,
      std::bind(&DriveArbitratorNode::pathStatusCallback, this, _1));
    follower_status_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      follower_status_topic_, 10,
      std::bind(&DriveArbitratorNode::followerStatusCallback, this, _1));
    guard_status_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      guard_status_topic_, 10,
      std::bind(&DriveArbitratorNode::guardStatusCallback, this, _1));
    reactive_status_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      reactive_status_topic_, 10,
      std::bind(&DriveArbitratorNode::reactiveStatusCallback, this, _1));
    lower_status_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      lower_status_topic_, 10,
      std::bind(&DriveArbitratorNode::lowerStatusCallback, this, _1));
    raceline_local_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      raceline_local_path_topic_, 10,
      std::bind(&DriveArbitratorNode::racelineLocalPathCallback, this, _1));
    reactive_local_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      reactive_local_path_topic_, 10,
      std::bind(&DriveArbitratorNode::reactiveLocalPathCallback, this, _1));
    pf_health_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      pf_health_topic_, 10,
      std::bind(&DriveArbitratorNode::pfHealthCallback, this, _1));
    reset_sub_ = create_subscription<std_msgs::msg::Bool>(
      reset_topic_, 10, std::bind(&DriveArbitratorNode::resetCallback, this, _1));

    selected_command_pub_ =
      create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(selected_command_topic_, 10);
    selected_mode_pub_ = create_publisher<std_msgs::msg::UInt8>(selected_mode_topic_, 10);
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(status_topic_, 10);
    ultimate_trajectory_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      ultimate_trajectory_topic_, 10);

    startup_started_at_ = steady_clock_.now();
    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DriveArbitratorNode::controlLoop, this));

    RCLCPP_INFO(
      get_logger(),
      "drive_arbitrator ready: raceline=%s reactive=%s output=%s require_pf=%s "
      "auto_recovery(blocked=%s,pf=%s,raceline_unavailable=%s,stable=%.2fs) "
      "lower_recovery_coordination=%s(timeout=%.2fs) ultimate_trajectory=%s",
      raceline_command_topic_.c_str(), reactive_command_topic_.c_str(),
      selected_command_topic_.c_str(), require_pf_health_ ? "true" : "false",
      allow_auto_recovery_from_blocked_ ? "true" : "false",
      allow_auto_recovery_from_pf_invalid_ ? "true" : "false",
      allow_auto_recovery_from_raceline_unavailable_ ? "true" : "false",
      raceline_recovery_stable_sec_,
      enable_lower_safety_recovery_coordination_ ? "true" : "false",
      lower_status_timeout_sec_,
      publish_ultimate_chosen_trajectory_ ? ultimate_trajectory_topic_.c_str() : "disabled");
  }

private:
  enum class Mode : uint8_t
  {
    WAITING = 0,
    RACELINE = 1,
    REACTIVE = 2,
    STOP = 3
  };

  struct StatusSample
  {
    bool received{false};
    std::string state;
    std::string reason;
    std::string arbitration_mode;
    std::string trajectory_mode;
    rclcpp::Time received_at{0, 0, RCL_STEADY_TIME};
  };

  struct Snapshot
  {
    ackermann_msgs::msg::AckermannDriveStamped::SharedPtr raceline_command;
    ackermann_msgs::msg::AckermannDriveStamped::SharedPtr reactive_command;
    bool raceline_command_received{false};
    bool reactive_command_received{false};
    rclcpp::Time raceline_command_time{0, 0, RCL_STEADY_TIME};
    rclcpp::Time reactive_command_time{0, 0, RCL_STEADY_TIME};
    StatusSample path;
    StatusSample follower;
    StatusSample guard;
    StatusSample reactive;
    StatusSample lower;
    nav_msgs::msg::Path::SharedPtr raceline_local_path;
    nav_msgs::msg::Path::SharedPtr reactive_local_path;
    bool raceline_local_path_received{false};
    bool reactive_local_path_received{false};
    rclcpp::Time raceline_local_path_time{0, 0, RCL_STEADY_TIME};
    rclcpp::Time reactive_local_path_time{0, 0, RCL_STEADY_TIME};
    bool pf_received{false};
    int pf_state{0};
    rclcpp::Time pf_time{0, 0, RCL_STEADY_TIME};
    bool reset_requested{false};
  };

  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
    raceline_command_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
    reactive_command_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr path_status_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr follower_status_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr guard_status_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr reactive_status_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr lower_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raceline_local_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr reactive_local_path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pf_health_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
    selected_command_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr selected_mode_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ultimate_trajectory_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  std::mutex mutex_;
  ackermann_msgs::msg::AckermannDriveStamped::SharedPtr latest_raceline_command_;
  ackermann_msgs::msg::AckermannDriveStamped::SharedPtr latest_reactive_command_;
  bool raceline_command_received_{false};
  bool reactive_command_received_{false};
  rclcpp::Time raceline_command_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time reactive_command_time_{0, 0, RCL_STEADY_TIME};
  StatusSample path_status_;
  StatusSample follower_status_;
  StatusSample guard_status_;
  StatusSample reactive_status_;
  StatusSample lower_status_;
  nav_msgs::msg::Path::SharedPtr latest_raceline_local_path_;
  nav_msgs::msg::Path::SharedPtr latest_reactive_local_path_;
  bool raceline_local_path_received_{false};
  bool reactive_local_path_received_{false};
  rclcpp::Time raceline_local_path_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time reactive_local_path_time_{0, 0, RCL_STEADY_TIME};
  bool pf_received_{false};
  int pf_state_{0};
  rclcpp::Time pf_time_{0, 0, RCL_STEADY_TIME};
  bool reset_requested_{false};

  std::string raceline_command_topic_;
  std::string reactive_command_topic_;
  std::string path_status_topic_;
  std::string follower_status_topic_;
  std::string guard_status_topic_;
  std::string reactive_status_topic_;
  std::string pf_health_topic_;
  std::string reset_topic_;
  std::string selected_command_topic_;
  std::string selected_mode_topic_;
  std::string status_topic_;
  std::string raceline_local_path_topic_;
  std::string reactive_local_path_topic_;
  std::string lower_status_topic_;
  std::string ultimate_trajectory_topic_;
  double control_rate_hz_{20.0};
  double primary_startup_timeout_sec_{3.0};
  double path_status_timeout_sec_{0.30};
  double follower_status_timeout_sec_{0.30};
  double guard_status_timeout_sec_{0.30};
  double raceline_command_timeout_sec_{0.25};
  double reactive_status_timeout_sec_{0.50};
  double reactive_command_timeout_sec_{0.30};
  double pf_health_timeout_sec_{0.50};
  bool require_pf_health_{true};
  bool latch_reactive_mode_{true};
  bool allow_auto_recovery_from_blocked_{false};
  bool allow_auto_recovery_from_pf_invalid_{false};
  bool allow_auto_recovery_from_raceline_unavailable_{false};
  double raceline_recovery_stable_sec_{1.0};
  bool enable_lower_safety_recovery_coordination_{true};
  double lower_status_timeout_sec_{0.30};
  bool publish_ultimate_chosen_trajectory_{true};
  double ultimate_trajectory_line_width_m_{0.05};
  double ultimate_trajectory_timeout_sec_{0.30};
  bool ultimate_trajectory_visible_{false};
  std::string last_ultimate_trajectory_frame_{"base_link"};

  Mode mode_{Mode::WAITING};
  Mode last_logged_mode_{Mode::WAITING};
  std::string mode_reason_{"waiting for inputs"};
  std::string last_logged_reason_;
  bool reactive_latched_{false};
  std::string latched_trigger_;
  bool saw_blocked_trigger_{false};
  bool saw_pf_trigger_{false};
  bool saw_raceline_unavailable_trigger_{false};
  bool raceline_recovery_pending_{false};
  rclcpp::Time raceline_recovery_started_at_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time startup_started_at_{0, 0, RCL_STEADY_TIME};

  void declareParameters()
  {
    declare_parameter<std::string>(
      "raceline_command_topic", "/path_following_v2/nominal_cmd");
    declare_parameter<std::string>(
      "reactive_command_topic", "/reactive_control_v2/nominal_cmd");
    declare_parameter<std::string>("path_status_topic", "/path_following_v2/path_status");
    declare_parameter<std::string>("follower_status_topic", "/path_following_v2/status");
    declare_parameter<std::string>(
      "guard_status_topic", "/drive_arbitration_v2/raceline_guard_status");
    declare_parameter<std::string>("reactive_status_topic", "/reactive_control_v2/status");
    declare_parameter<std::string>("pf_health_topic", "/pf/health");
    declare_parameter<std::string>("reset_topic", "/drive_arbitration_v2/reset");
    declare_parameter<std::string>(
      "selected_command_topic", "/drive_arbitration_v2/selected_cmd");
    declare_parameter<std::string>(
      "selected_mode_topic", "/drive_arbitration_v2/selected_mode");
    declare_parameter<std::string>("status_topic", "/drive_arbitration_v2/status");
    declare_parameter<std::string>(
      "raceline_local_path_topic", "/path_following_v2/local_path");
    declare_parameter<std::string>(
      "reactive_local_path_topic", "/reactive_control_v2/local_path");
    declare_parameter<std::string>(
      "lower_status_topic", "/reactive_control_v2/lower_safety_status");
    declare_parameter<std::string>(
      "ultimate_trajectory_topic",
      "/drive_arbitration_v2/ultimate_chosen_local_trajectory");
    declare_parameter<double>("control_rate_hz", 20.0);
    declare_parameter<double>("primary_startup_timeout_sec", 3.0);
    declare_parameter<double>("path_status_timeout_sec", 0.30);
    declare_parameter<double>("follower_status_timeout_sec", 0.30);
    declare_parameter<double>("guard_status_timeout_sec", 0.30);
    declare_parameter<double>("raceline_command_timeout_sec", 0.25);
    declare_parameter<double>("reactive_status_timeout_sec", 0.50);
    declare_parameter<double>("reactive_command_timeout_sec", 0.30);
    declare_parameter<double>("pf_health_timeout_sec", 0.50);
    declare_parameter<bool>("require_pf_health", true);
    declare_parameter<bool>("latch_reactive_mode", true);
    declare_parameter<bool>("allow_auto_recovery_from_blocked", false);
    declare_parameter<bool>("allow_auto_recovery_from_pf_invalid", false);
    declare_parameter<bool>("allow_auto_recovery_from_raceline_unavailable", false);
    declare_parameter<double>("raceline_recovery_stable_sec", 1.0);
    declare_parameter<bool>("enable_lower_safety_recovery_coordination", true);
    declare_parameter<double>("lower_status_timeout_sec", 0.30);
    declare_parameter<bool>("publish_ultimate_chosen_trajectory", true);
    declare_parameter<double>("ultimate_trajectory_line_width_m", 0.05);
    declare_parameter<double>("ultimate_trajectory_timeout_sec", 0.30);
  }

  void loadParameters()
  {
    raceline_command_topic_ = get_parameter("raceline_command_topic").as_string();
    reactive_command_topic_ = get_parameter("reactive_command_topic").as_string();
    path_status_topic_ = get_parameter("path_status_topic").as_string();
    follower_status_topic_ = get_parameter("follower_status_topic").as_string();
    guard_status_topic_ = get_parameter("guard_status_topic").as_string();
    reactive_status_topic_ = get_parameter("reactive_status_topic").as_string();
    pf_health_topic_ = get_parameter("pf_health_topic").as_string();
    reset_topic_ = get_parameter("reset_topic").as_string();
    selected_command_topic_ = get_parameter("selected_command_topic").as_string();
    selected_mode_topic_ = get_parameter("selected_mode_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    raceline_local_path_topic_ = get_parameter("raceline_local_path_topic").as_string();
    reactive_local_path_topic_ = get_parameter("reactive_local_path_topic").as_string();
    lower_status_topic_ = get_parameter("lower_status_topic").as_string();
    ultimate_trajectory_topic_ = get_parameter("ultimate_trajectory_topic").as_string();
    control_rate_hz_ = std::max(1.0, get_parameter("control_rate_hz").as_double());
    primary_startup_timeout_sec_ = std::max(
      0.0, get_parameter("primary_startup_timeout_sec").as_double());
    path_status_timeout_sec_ = std::max(
      0.01, get_parameter("path_status_timeout_sec").as_double());
    follower_status_timeout_sec_ = std::max(
      0.01, get_parameter("follower_status_timeout_sec").as_double());
    guard_status_timeout_sec_ = std::max(
      0.01, get_parameter("guard_status_timeout_sec").as_double());
    raceline_command_timeout_sec_ = std::max(
      0.01, get_parameter("raceline_command_timeout_sec").as_double());
    reactive_status_timeout_sec_ = std::max(
      0.01, get_parameter("reactive_status_timeout_sec").as_double());
    reactive_command_timeout_sec_ = std::max(
      0.01, get_parameter("reactive_command_timeout_sec").as_double());
    pf_health_timeout_sec_ = std::max(
      0.01, get_parameter("pf_health_timeout_sec").as_double());
    require_pf_health_ = get_parameter("require_pf_health").as_bool();
    latch_reactive_mode_ = get_parameter("latch_reactive_mode").as_bool();
    allow_auto_recovery_from_blocked_ =
      get_parameter("allow_auto_recovery_from_blocked").as_bool();
    allow_auto_recovery_from_pf_invalid_ =
      get_parameter("allow_auto_recovery_from_pf_invalid").as_bool();
    allow_auto_recovery_from_raceline_unavailable_ =
      get_parameter("allow_auto_recovery_from_raceline_unavailable").as_bool();
    raceline_recovery_stable_sec_ = std::max(
      0.0, get_parameter("raceline_recovery_stable_sec").as_double());
    enable_lower_safety_recovery_coordination_ =
      get_parameter("enable_lower_safety_recovery_coordination").as_bool();
    lower_status_timeout_sec_ = std::max(
      0.01, get_parameter("lower_status_timeout_sec").as_double());
    publish_ultimate_chosen_trajectory_ =
      get_parameter("publish_ultimate_chosen_trajectory").as_bool();
    ultimate_trajectory_line_width_m_ = std::max(
      0.001, get_parameter("ultimate_trajectory_line_width_m").as_double());
    ultimate_trajectory_timeout_sec_ = std::max(
      0.01, get_parameter("ultimate_trajectory_timeout_sec").as_double());
  }

  void racelineCommandCallback(
    const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr command)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_raceline_command_ = command;
    raceline_command_time_ = steady_clock_.now();
    raceline_command_received_ = true;
  }

  void reactiveCommandCallback(
    const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr command)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_reactive_command_ = command;
    reactive_command_time_ = steady_clock_.now();
    reactive_command_received_ = true;
  }

  bool parseStatus(
    const diagnostic_msgs::msg::DiagnosticArray & array,
    const std::string & expected_name, StatusSample & destination)
  {
    for (const auto & status : array.status) {
      if (status.name != expected_name) {
        continue;
      }
      destination.state.clear();
      destination.reason.clear();
      destination.arbitration_mode.clear();
      destination.trajectory_mode.clear();
      for (const auto & value : status.values) {
        if (value.key == "state") {
          destination.state = value.value;
        } else if (value.key == "mode" && destination.state.empty()) {
          destination.state = value.value;
        } else if (value.key == "reason") {
          destination.reason = value.value;
        } else if (value.key == "arbitration_mode") {
          destination.arbitration_mode = value.value;
        } else if (value.key == "trajectory_mode") {
          destination.trajectory_mode = value.value;
        }
      }
      if (destination.state.empty()) {
        const auto separator = status.message.find(':');
        destination.state = status.message.substr(0, separator);
      }
      if (destination.reason.empty()) {
        destination.reason = status.message;
      }
      destination.received = true;
      destination.received_at = steady_clock_.now();
      return true;
    }
    return false;
  }

  void pathStatusCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr array)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    parseStatus(*array, "path_following_v2/path_generator", path_status_);
  }

  void followerStatusCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr array)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    parseStatus(*array, "path_following_v2/path_follower", follower_status_);
  }

  void guardStatusCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr array)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    parseStatus(*array, "drive_arbitration_v2/raceline_guard", guard_status_);
  }

  void reactiveStatusCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr array)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    parseStatus(*array, "reactive_control_v2/upper_corridor_follower", reactive_status_);
  }

  void lowerStatusCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr array)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    parseStatus(*array, "reactive_control_v2/lower_safety_controller", lower_status_);
  }

  void racelineLocalPathCallback(const nav_msgs::msg::Path::SharedPtr path)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_raceline_local_path_ = path;
    raceline_local_path_time_ = steady_clock_.now();
    raceline_local_path_received_ = true;
  }

  void reactiveLocalPathCallback(const nav_msgs::msg::Path::SharedPtr path)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_reactive_local_path_ = path;
    reactive_local_path_time_ = steady_clock_.now();
    reactive_local_path_received_ = true;
  }

  void pfHealthCallback(const std_msgs::msg::Float32MultiArray::SharedPtr health)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pf_received_ = true;
    pf_time_ = steady_clock_.now();
    if (health->data.empty() || !std::isfinite(health->data.front())) {
      pf_state_ = 0;
      return;
    }
    pf_state_ = static_cast<int>(std::lround(health->data.front()));
  }

  void resetCallback(const std_msgs::msg::Bool::SharedPtr reset)
  {
    if (!reset->data) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    reset_requested_ = true;
  }

  Snapshot snapshot()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Snapshot result;
    result.raceline_command = latest_raceline_command_;
    result.reactive_command = latest_reactive_command_;
    result.raceline_command_received = raceline_command_received_;
    result.reactive_command_received = reactive_command_received_;
    result.raceline_command_time = raceline_command_time_;
    result.reactive_command_time = reactive_command_time_;
    result.path = path_status_;
    result.follower = follower_status_;
    result.guard = guard_status_;
    result.reactive = reactive_status_;
    result.lower = lower_status_;
    result.raceline_local_path = latest_raceline_local_path_;
    result.reactive_local_path = latest_reactive_local_path_;
    result.raceline_local_path_received = raceline_local_path_received_;
    result.reactive_local_path_received = reactive_local_path_received_;
    result.raceline_local_path_time = raceline_local_path_time_;
    result.reactive_local_path_time = reactive_local_path_time_;
    result.pf_received = pf_received_;
    result.pf_state = pf_state_;
    result.pf_time = pf_time_;
    result.reset_requested = reset_requested_;
    reset_requested_ = false;
    return result;
  }

  double age(const bool received, const rclcpp::Time & time, const rclcpp::Time & now) const
  {
    return received ? (now - time).seconds() : std::numeric_limits<double>::infinity();
  }

  bool commandValid(
    const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr & command) const
  {
    return command && std::isfinite(command->drive.speed) &&
           std::isfinite(command->drive.steering_angle);
  }

  const char * modeName(const Mode mode) const
  {
    switch (mode) {
      case Mode::WAITING: return "WAITING";
      case Mode::RACELINE: return "RACELINE";
      case Mode::REACTIVE: return "REACTIVE";
      case Mode::STOP: return "STOP";
    }
    return "STOP";
  }

  std::string racelineUnavailableReason(
    const Snapshot & input, const rclcpp::Time & current_time) const
  {
    if (require_pf_health_) {
      if (!input.pf_received ||
        age(input.pf_received, input.pf_time, current_time) > pf_health_timeout_sec_)
      {
        return "PF_UNAVAILABLE: health message missing or stale";
      }
      if (input.pf_state == 3) {
        return "PF_INVALID: particle-filter health state is 3";
      }
      if (input.pf_state != 1 && input.pf_state != 2) {
        return "PF_UNAVAILABLE: malformed or unknown health state";
      }
    }
    if (!input.path.received ||
      age(input.path.received, input.path.received_at, current_time) > path_status_timeout_sec_)
    {
      return "RACELINE_UNAVAILABLE: path-generator status missing or stale";
    }
    if (input.path.state == "NO_SAFE_PATH_CONFIRMED" ||
      input.path.state == "NO_SAFE_DETOUR" ||
      input.path.state == "CRITICAL_OBSTACLE")
    {
      // These are obstacle outcomes, not generic raceline-data failures. Keep
      // them in the existing blockage trigger class so the blockage recovery
      // policy and diagnostics remain meaningful.
      return "RACELINE_BLOCKED: trajectory planner state " + input.path.state +
        ": " + input.path.reason;
    }
    if (input.path.state != "READY") {
      return "RACELINE_UNAVAILABLE: primary trajectory planner state " + input.path.state;
    }
    if (!input.follower.received ||
      age(input.follower.received, input.follower.received_at, current_time) >
      follower_status_timeout_sec_)
    {
      return "RACELINE_UNAVAILABLE: follower status missing or stale";
    }
    if (input.follower.state != "DRIVING") {
      return "RACELINE_UNAVAILABLE: follower state " + input.follower.state;
    }
    if (!input.raceline_command_received ||
      age(input.raceline_command_received, input.raceline_command_time, current_time) >
      raceline_command_timeout_sec_)
    {
      return "RACELINE_UNAVAILABLE: raceline candidate command missing or stale";
    }
    if (!commandValid(input.raceline_command)) {
      return "RACELINE_UNAVAILABLE: raceline candidate command is non-finite";
    }
    if (!input.guard.received ||
      age(input.guard.received, input.guard.received_at, current_time) > guard_status_timeout_sec_)
    {
      return "RACELINE_UNAVAILABLE: raceline guard status missing or stale";
    }
    if (input.guard.state == "BLOCKED") {
      const bool critical_guard = input.guard.reason.find("critical") != std::string::npos;
      const bool local_replan_executing =
        input.path.trajectory_mode == "AVOIDING" ||
        input.path.trajectory_mode == "REJOINING" ||
        input.path.trajectory_mode == "RECOVERING_TO_RACELINE" ||
        input.path.trajectory_mode == "REPLAN_PENDING";
      if (local_replan_executing && !critical_guard && !reactive_latched_) {
        // The local planner applies the same widened-band test and confirms a
        // blocked active plan over fresh scans.  Let it slow/replan instead of
        // latching Reactive from one transient guard result.  Once Reactive is
        // already latched, guard CLEAR is still required for recovery.
        return "";
      }
      return "RACELINE_BLOCKED: " + input.guard.reason;
    }
    if (input.guard.state != "CLEAR") {
      return "RACELINE_UNAVAILABLE: raceline guard state " + input.guard.state;
    }
    return "";
  }

  bool reactiveChainAvailable(
    const Snapshot & input, const rclcpp::Time & current_time) const
  {
    if (!input.reactive.received ||
      age(input.reactive.received, input.reactive.received_at, current_time) >
      reactive_status_timeout_sec_)
    {
      return false;
    }
    if (input.reactive.state == "PATH_INVALID" || input.reactive.state == "BLOCKED") {
      // The lower controller may use FTG, but only after selected_mode=REACTIVE.
      return true;
    }
    if (input.reactive.state != "DRIVING" &&
      input.reactive.state != "PATH_VALIDATION_PENDING")
    {
      return false;
    }
    return input.reactive_command_received && commandValid(input.reactive_command) &&
      age(input.reactive_command_received, input.reactive_command_time, current_time) <=
      reactive_command_timeout_sec_;
  }

  void recordFallbackTrigger(const std::string & reason)
  {
    if (reason.rfind("RACELINE_BLOCKED:", 0) == 0) {
      saw_blocked_trigger_ = true;
    } else if (reason.rfind("PF_INVALID:", 0) == 0 ||
      reason.rfind("PF_UNAVAILABLE:", 0) == 0)
    {
      // The PF recovery switch covers invalid, missing, stale, malformed, and
      // unknown PF health because all make localization unusable.
      saw_pf_trigger_ = true;
    } else {
      saw_raceline_unavailable_trigger_ = true;
    }
  }

  bool automaticRecoveryAllowed() const
  {
    const bool has_recorded_trigger =
      saw_blocked_trigger_ || saw_pf_trigger_ || saw_raceline_unavailable_trigger_;
    return has_recorded_trigger &&
      (!saw_blocked_trigger_ || allow_auto_recovery_from_blocked_) &&
      (!saw_pf_trigger_ || allow_auto_recovery_from_pf_invalid_) &&
      (!saw_raceline_unavailable_trigger_ ||
      allow_auto_recovery_from_raceline_unavailable_);
  }

  bool lowerStatusFresh(
    const Snapshot & input, const rclcpp::Time & current_time) const
  {
    return input.lower.received &&
      age(input.lower.received, input.lower.received_at, current_time) <=
      lower_status_timeout_sec_;
  }

  std::string lowerRecoveryHoldReason(
    const Snapshot & input, const rclcpp::Time & current_time) const
  {
    if (!enable_lower_safety_recovery_coordination_) {
      return "";
    }
    if (!lowerStatusFresh(input, current_time)) {
      return "lower safety status missing or stale";
    }
    // The lower status reports the arbitrator's UInt8 mode as text.  Requiring
    // mode 2 prevents a delayed NOMINAL sample from another arbitration mode
    // from authorizing a raceline handover.
    if (input.lower.arbitration_mode != "2") {
      return "lower safety has not confirmed REACTIVE arbitration mode";
    }
    if (input.lower.state != "NOMINAL") {
      return "lower safety mode=" + input.lower.state;
    }
    return "";
  }

  bool lowerEmergencyUnderRaceline(
    const Snapshot & input, const rclcpp::Time & current_time) const
  {
    // This safety net covers the short race in which the raceline guard clears,
    // RACELINE is selected, and the final lower layer then detects a hard-front
    // emergency.  Mode feedback must confirm 1 (RACELINE), so stale emergency
    // status from the previous Reactive episode cannot retrigger fallback.
    return enable_lower_safety_recovery_coordination_ &&
      lowerStatusFresh(input, current_time) &&
      input.lower.state == "EMERGENCY_STOP" &&
      input.lower.arbitration_mode == "1";
  }

  std::string fallbackTriggerSummary() const
  {
    std::string result;
    auto append = [&result](const std::string & value) {
        if (!result.empty()) {
          result += ",";
        }
        result += value;
      };
    if (saw_blocked_trigger_) {
      append("RACELINE_BLOCKED");
    }
    if (saw_pf_trigger_) {
      append("PF_INVALID_OR_UNAVAILABLE");
    }
    if (saw_raceline_unavailable_trigger_) {
      append("RACELINE_UNAVAILABLE");
    }
    return result.empty() ? "none" : result;
  }

  void clearReactiveLatch()
  {
    reactive_latched_ = false;
    latched_trigger_.clear();
    saw_blocked_trigger_ = false;
    saw_pf_trigger_ = false;
    saw_raceline_unavailable_trigger_ = false;
    raceline_recovery_pending_ = false;
  }

  ackermann_msgs::msg::AckermannDriveStamped stopCommand() const
  {
    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header.stamp = now();
    command.header.frame_id = "base_link";
    command.drive.speed = 0.0;
    command.drive.steering_angle = 0.0;
    return command;
  }

  void logTransition()
  {
    if (mode_ == last_logged_mode_ && mode_reason_ == last_logged_reason_) {
      return;
    }
    if (mode_ == Mode::RACELINE) {
      RCLCPP_INFO(get_logger(), "selected RACELINE: %s", mode_reason_.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(), "selected %s: %s", modeName(mode_), mode_reason_.c_str());
    }
    last_logged_mode_ = mode_;
    last_logged_reason_ = mode_reason_;
  }

  void publishStatus(
    const Snapshot & input, const rclcpp::Time & current_time,
    const bool primary_ready, const bool reactive_ready,
    const std::string & primary_failure)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "drive_arbitration_v2/drive_arbitrator";
    status.hardware_id = "controller_selector";
    status.level = mode_ == Mode::RACELINE || mode_ == Mode::REACTIVE ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = std::string(modeName(mode_)) + ": " + mode_reason_;

    auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue pair;
        pair.key = key;
        pair.value = value;
        status.values.push_back(pair);
      };
    add("mode", modeName(mode_));
    add("mode_code", std::to_string(static_cast<uint8_t>(mode_)));
    add("reason", mode_reason_);
    add("primary_ready", primary_ready ? "true" : "false");
    add("primary_failure", primary_failure.empty() ? "none" : primary_failure);
    add("reactive_ready", reactive_ready ? "true" : "false");
    add("reactive_latched", reactive_latched_ ? "true" : "false");
    add("latched_trigger_classes", fallbackTriggerSummary());
    add("auto_recovery_allowed", automaticRecoveryAllowed() ? "true" : "false");
    add("raceline_recovery_pending", raceline_recovery_pending_ ? "true" : "false");
    add(
      "raceline_recovery_elapsed_sec",
      raceline_recovery_pending_ ?
      std::to_string((current_time - raceline_recovery_started_at_).seconds()) : "0.0");
    add("raceline_recovery_stable_sec", std::to_string(raceline_recovery_stable_sec_));
    add(
      "lower_recovery_coordination_enabled",
      enable_lower_safety_recovery_coordination_ ? "true" : "false");
    add("lower_status_state", input.lower.state);
    add("lower_status_arbitration_mode", input.lower.arbitration_mode);
    add(
      "lower_status_age_sec",
      std::to_string(age(input.lower.received, input.lower.received_at, current_time)));
    add(
      "lower_allows_raceline_recovery",
      lowerRecoveryHoldReason(input, current_time).empty() ? "true" : "false");
    add("path_generator_state", input.path.state);
    add(
      "primary_trajectory_mode",
      input.path.trajectory_mode.empty() ? "UNKNOWN" : input.path.trajectory_mode);
    add("follower_state", input.follower.state);
    add("guard_state", input.guard.state);
    add("reactive_upper_state", input.reactive.state);
    add("pf_required", require_pf_health_ ? "true" : "false");
    add("pf_state", std::to_string(input.pf_state));
    add(
      "raceline_command_age_sec",
      std::to_string(age(
        input.raceline_command_received, input.raceline_command_time, current_time)));
    add(
      "reactive_command_age_sec",
      std::to_string(age(
        input.reactive_command_received, input.reactive_command_time, current_time)));
    array.status.push_back(status);
    status_pub_->publish(array);
  }

  void hideUltimateTrajectory()
  {
    if (!ultimate_trajectory_visible_) {
      return;
    }
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = last_ultimate_trajectory_frame_;
    marker.ns = "ultimate_chosen_local_trajectory";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    ultimate_trajectory_pub_->publish(marker);
    ultimate_trajectory_visible_ = false;
  }

  void publishUltimateTrajectory(
    const Snapshot & input, const rclcpp::Time & current_time)
  {
    if (!publish_ultimate_chosen_trajectory_) {
      hideUltimateTrajectory();
      return;
    }

    const bool lower_status_fresh = input.lower.received &&
      age(input.lower.received, input.lower.received_at, current_time) <=
      ultimate_trajectory_timeout_sec_;
    const std::string expected_mode = std::to_string(static_cast<uint8_t>(mode_));
    const bool lower_follows_selected_nominal = lower_status_fresh &&
      input.lower.state == "NOMINAL" && input.lower.arbitration_mode == expected_mode;

    nav_msgs::msg::Path::SharedPtr selected_path;
    bool path_received = false;
    rclcpp::Time path_time(0, 0, RCL_STEADY_TIME);
    bool show_raceline = false;

    if (lower_follows_selected_nominal && mode_ == Mode::RACELINE) {
      selected_path = input.raceline_local_path;
      path_received = input.raceline_local_path_received;
      path_time = input.raceline_local_path_time;
      show_raceline = true;
    } else if (lower_follows_selected_nominal && mode_ == Mode::REACTIVE) {
      selected_path = input.reactive_local_path;
      path_received = input.reactive_local_path_received;
      path_time = input.reactive_local_path_time;
    } else {
      hideUltimateTrajectory();
      return;
    }

    const bool path_fresh = path_received &&
      age(path_received, path_time, current_time) <= ultimate_trajectory_timeout_sec_;
    if (!path_fresh || !selected_path || selected_path->header.frame_id.empty() ||
      selected_path->poses.size() < 2)
    {
      hideUltimateTrajectory();
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header = selected_path->header;
    marker.ns = "ultimate_chosen_local_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = ultimate_trajectory_line_width_m_;
    marker.color.a = 1.0F;
    const bool show_local_replan = show_raceline &&
      (input.path.trajectory_mode == "AVOIDING" ||
      input.path.trajectory_mode == "REJOINING" ||
      input.path.trajectory_mode == "RECOVERING_TO_RACELINE" ||
      input.path.trajectory_mode == "REPLAN_PENDING");
    if (show_local_replan) {
      // Light blue means this is the localization-based local replan that is
      // actually authorized through the lower controller right now.
      marker.color.r = 0.20F;
      marker.color.g = 0.80F;
      marker.color.b = 1.00F;
    } else if (show_raceline) {
      marker.color.r = 0.0F;
      marker.color.g = 0.15F;
      marker.color.b = 0.75F;
    } else {
      marker.color.r = 1.0F;
      marker.color.g = 0.5F;
      marker.color.b = 0.0F;
    }
    marker.points.reserve(selected_path->poses.size());
    for (const auto & pose : selected_path->poses) {
      marker.points.push_back(pose.pose.position);
    }

    last_ultimate_trajectory_frame_ = marker.header.frame_id;
    ultimate_trajectory_pub_->publish(marker);
    ultimate_trajectory_visible_ = true;
  }

  void controlLoop()
  {
    Snapshot input = snapshot();
    const auto current_time = steady_clock_.now();
    if (input.reset_requested) {
      clearReactiveLatch();
      mode_ = Mode::WAITING;
      mode_reason_ = "manual reset received; waiting for primary inputs";
      startup_started_at_ = current_time;
    }

    const std::string primary_failure = racelineUnavailableReason(input, current_time);
    const bool primary_ready = primary_failure.empty();
    const bool reactive_ready = reactiveChainAvailable(input, current_time);

    const bool lower_emergency_fallback =
      mode_ == Mode::RACELINE && lowerEmergencyUnderRaceline(input, current_time);
    if (lower_emergency_fallback) {
      // The lower controller has final /drive authority.  Keep its emergency
      // stop, but reselect/latch REACTIVE so its existing dead-end timer, FTG,
      // and reverse recovery are authorized on subsequent cycles.
      const std::string trigger =
        "RACELINE_BLOCKED: lower safety emergency while raceline selected";
      reactive_latched_ = true;
      latched_trigger_ = trigger;
      recordFallbackTrigger(trigger);
      raceline_recovery_pending_ = false;
    }

    if (reactive_latched_) {
      if (!primary_ready) {
        recordFallbackTrigger(primary_failure);
        raceline_recovery_pending_ = false;
        if (reactive_ready) {
          mode_ = Mode::REACTIVE;
          mode_reason_ = primary_failure;
        } else {
          mode_ = Mode::STOP;
          mode_reason_ = primary_failure + "; reactive chain unavailable";
        }
      } else if (automaticRecoveryAllowed()) {
        const std::string lower_hold_reason = lowerRecoveryHoldReason(input, current_time);
        if (!lower_hold_reason.empty()) {
          // Reset rather than pause: handover requires one uninterrupted
          // interval with both the primary chain healthy and the lower layer
          // confirmed NOMINAL in REACTIVE mode.
          raceline_recovery_pending_ = false;
          if (reactive_ready) {
            mode_ = Mode::REACTIVE;
            mode_reason_ = lower_emergency_fallback ?
              "RACELINE_BLOCKED: lower safety emergency while raceline selected" :
              "automatic raceline recovery held; " + lower_hold_reason;
          } else {
            mode_ = Mode::STOP;
            mode_reason_ = "automatic raceline recovery held; " + lower_hold_reason +
              "; reactive chain unavailable";
          }
        } else {
          if (!raceline_recovery_pending_) {
            raceline_recovery_pending_ = true;
            raceline_recovery_started_at_ = current_time;
          }
          const double stable_age =
            (current_time - raceline_recovery_started_at_).seconds();
          if (stable_age >= raceline_recovery_stable_sec_) {
            const std::string recovered_triggers = fallbackTriggerSummary();
            clearReactiveLatch();
            mode_ = Mode::RACELINE;
            mode_reason_ =
              "automatic return after stable healthy/clear raceline and lower NOMINAL; "
              "recovered from " + recovered_triggers;
          } else if (reactive_ready) {
            mode_ = Mode::REACTIVE;
            mode_reason_ = "automatic raceline recovery pending; stability timer running";
          } else {
            mode_ = Mode::STOP;
            mode_reason_ = "automatic raceline recovery pending; reactive chain unavailable";
          }
        }
      } else if (reactive_ready) {
        raceline_recovery_pending_ = false;
        mode_ = Mode::REACTIVE;
        mode_reason_ = latched_trigger_.empty() ?
          "reactive fallback remains latched; automatic recovery disabled" :
          latched_trigger_ + "; automatic recovery disabled for recorded trigger";
      } else {
        raceline_recovery_pending_ = false;
        mode_ = Mode::STOP;
        mode_reason_ = "reactive fallback latched but reactive chain is unavailable";
      }
    } else if (mode_ == Mode::RACELINE && !primary_ready) {
      reactive_latched_ = latch_reactive_mode_;
      latched_trigger_ = primary_failure;
      recordFallbackTrigger(primary_failure);
      if (reactive_ready) {
        mode_ = Mode::REACTIVE;
        mode_reason_ = primary_failure;
      } else {
        mode_ = Mode::STOP;
        mode_reason_ = primary_failure + "; reactive chain unavailable";
      }
    } else if (primary_ready) {
      mode_ = Mode::RACELINE;
      mode_reason_ = "raceline chain, guard, and required localization are healthy";
    } else {
      const double startup_age = (current_time - startup_started_at_).seconds();
      if (startup_age < primary_startup_timeout_sec_) {
        mode_ = Mode::WAITING;
        mode_reason_ = "startup grace: " + primary_failure;
      } else {
        reactive_latched_ = latch_reactive_mode_;
        latched_trigger_ = primary_failure;
        recordFallbackTrigger(primary_failure);
        if (reactive_ready) {
          mode_ = Mode::REACTIVE;
          mode_reason_ = primary_failure;
        } else {
          mode_ = Mode::STOP;
          mode_reason_ = primary_failure + "; reactive chain unavailable";
        }
      }
    }

    ackermann_msgs::msg::AckermannDriveStamped selected = stopCommand();
    if (mode_ == Mode::RACELINE && commandValid(input.raceline_command)) {
      selected = *input.raceline_command;
      selected.header.stamp = now();
    } else if (mode_ == Mode::REACTIVE && commandValid(input.reactive_command) &&
      age(input.reactive_command_received, input.reactive_command_time, current_time) <=
      reactive_command_timeout_sec_)
    {
      selected = *input.reactive_command;
      selected.header.stamp = now();
    }

    std_msgs::msg::UInt8 mode_message;
    mode_message.data = static_cast<uint8_t>(mode_);
    selected_mode_pub_->publish(mode_message);
    selected_command_pub_->publish(selected);
    publishUltimateTrajectory(input, current_time);
    publishStatus(input, current_time, primary_ready, reactive_ready, primary_failure);
    logTransition();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveArbitratorNode>());
  rclcpp::shutdown();
  return 0;
}
