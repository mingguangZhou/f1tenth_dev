import math
from typing import Tuple

import gym
import numpy as np

from rl_training.centerline_utils import (
    load_centerline_csv,
    get_centerline_state_features,
    get_upcoming_curvature_abs,
    compute_centerline_progress_delta,
)
from rl_training.pure_pursuit import compute_pure_pursuit_steering
from rl_training.speed_policies import curvature_based_speed


class F110SpeedEnv(gym.Env):
    """
    Speed-only residual RL environment for F1TENTH / RoboRacer training.

    Steering is NOT learned. Steering stays rule-based pure pursuit, matching the
    current ROS 2 path_following_v2 pipeline.

    Longitudinal architecture:
        rule_speed_mps      = curvature_based_speed(rule_curvature_abs, rule_min/max_speed_mps)
        delta_speed_mps     = max_delta_speed_mps * correction_action
        requested_speed_mps = rule_speed_mps + delta_speed_mps
        target_speed_mps    = rate_limited_and_clipped(requested_speed_mps)

    The rule-based speed uses a deliberately short and simple curvature preview.
    The RL observation gets three curvature preview layers so the policy can
    learn anticipatory corrections without making the rule-based reference
    complicated.

    Optional training disturbances:
        - random_start_along_centerline: reset at random raceline indices.
          This is safe because the base pose is exactly on the generated raceline.
          Optional lateral/yaw noise is clipped to small values.
        - observation noise: perturb only the RL observation, not the simulator
          state and not the reward calculation. Reward still uses the true state.

    RL action:
        [correction_action] in [-1.0, 1.0]

    RL observation:
        [
            current_speed_mps,
            rule_speed_mps,
            cross_track_error_obs,
            heading_error_obs,
            curv_short_abs,
            curv_mid_abs,
            curv_long_abs,
            previous_delta_speed_mps,
        ]
    """

    def __init__(
        self,
        map_path: str,
        map_ext: str,
        centerline_csv: str,
        start_pose: Tuple[float, float, float],
        lookahead_distance: float = 1.0,
        wheelbase: float = 0.33,
        max_steer: float = 0.4189,
        min_speed: float = 0.5,
        max_speed: float = 4.0,
        rule_min_speed_mps: float = None,
        rule_max_speed_mps: float = None,
        max_speed_index_delta: float = 0.05,  # kept for backward CLI compatibility
        max_speed_delta_per_step_mps: float = 0.10,
        max_delta_speed_mps: float = 0.30,
        residual_correction_scale: float = 0.25,  # deprecated; not used by physical residual mode
        max_episode_steps: int = 5000,
        lap_completion_ratio: float = 0.995,
        lap_bonus: float = 500.0,
        target_lap_steps: int = 5000,
        curvature_gain: float = 2.0,
        rule_curvature_lookahead_points: int = 3,
        model_curvature_short_points: int = 10,
        model_curvature_mid_points: int = 40,
        model_curvature_long_points: int = 80,
        random_start_along_centerline: bool = False,
        random_start_min_index: int = -1,
        random_start_max_index: int = -1,
        start_lateral_noise_std: float = 0.0,
        start_lateral_noise_max: float = 0.05,
        start_yaw_noise_std: float = 0.0,
        start_yaw_noise_max: float = 0.05,
        start_xy_noise_std: float = 0.0,
        start_xy_noise_max: float = 0.05,
        obs_cte_noise_std: float = 0.0,
        obs_heading_noise_std: float = 0.0,
        obs_speed_noise_std: float = 0.0,
        enable_bad_tracking_termination: bool = False,
        bad_tracking_min_steps: int = 50,
        bad_tracking_cte_threshold: float = 0.75,
        bad_tracking_heading_threshold: float = 0.90,
        crash_penalty_value: float = 1000.0,
        timeout_penalty_value: float = 500.0,
        target_speed_smoothness_weight: float = 0.04,
        random_seed: int = None,
        use_speed_dependent_lookahead: bool = True,
        min_lookahead: float = 0.6,
        max_lookahead: float = 1.6,
        lookahead_speed_gain: float = 0.25,
    ):
        super().__init__()

        if max_speed <= min_speed:
            raise ValueError(
                f"max_speed must be larger than min_speed. Got min_speed={min_speed}, max_speed={max_speed}."
            )
        if max_delta_speed_mps < 0.0:
            raise ValueError("max_delta_speed_mps must be non-negative")

        self.map_path = map_path
        self.map_ext = map_ext
        self.centerline = load_centerline_csv(centerline_csv)
        self.base_start_pose = np.array(start_pose, dtype=np.float32)
        self.start_pose = np.array([start_pose], dtype=np.float32)

        self.lookahead_distance = lookahead_distance
        self.wheelbase = wheelbase
        self.max_steer = max_steer
        self.use_speed_dependent_lookahead = use_speed_dependent_lookahead
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.lookahead_speed_gain = lookahead_speed_gain

        self.min_speed = float(min_speed)
        self.max_speed = float(max_speed)

        # The final commanded speed is clamped by [min_speed, max_speed].
        # The curvature-rule reference can have its own conservative range.
        # Example: physical max_speed=7.0, rule_max_speed_mps=5.8,
        # max_delta_speed_mps=1.2 allows RL to explore up to 7.0 on safe sections.
        self.rule_min_speed_mps = self.min_speed if rule_min_speed_mps is None else float(rule_min_speed_mps)
        self.rule_max_speed_mps = self.max_speed if rule_max_speed_mps is None else float(rule_max_speed_mps)
        self.rule_min_speed_mps = float(np.clip(self.rule_min_speed_mps, self.min_speed, self.max_speed))
        self.rule_max_speed_mps = float(np.clip(self.rule_max_speed_mps, self.rule_min_speed_mps, self.max_speed))

        self.max_speed_index_delta = float(max_speed_index_delta)
        self.max_speed_delta_per_step_mps = float(max_speed_delta_per_step_mps)
        self.max_delta_speed_mps = float(max_delta_speed_mps)
        self.residual_correction_scale = float(residual_correction_scale)

        self.max_episode_steps = int(max_episode_steps)
        self.lap_completion_ratio = float(lap_completion_ratio)
        self.lap_bonus = float(lap_bonus)
        self.target_lap_steps = int(target_lap_steps)
        self.curvature_gain = float(curvature_gain)

        self.rule_curvature_lookahead_points = int(rule_curvature_lookahead_points)
        self.model_curvature_short_points = int(model_curvature_short_points)
        self.model_curvature_mid_points = int(model_curvature_mid_points)
        self.model_curvature_long_points = int(model_curvature_long_points)

        self.random_start_along_centerline = bool(random_start_along_centerline)
        self.random_start_min_index = int(random_start_min_index)
        self.random_start_max_index = int(random_start_max_index)
        self.start_lateral_noise_std = float(start_lateral_noise_std)
        self.start_lateral_noise_max = abs(float(start_lateral_noise_max))
        self.start_yaw_noise_std = float(start_yaw_noise_std)
        self.start_yaw_noise_max = abs(float(start_yaw_noise_max))
        self.start_xy_noise_std = float(start_xy_noise_std)
        self.start_xy_noise_max = abs(float(start_xy_noise_max))

        self.obs_cte_noise_std = float(obs_cte_noise_std)
        self.obs_heading_noise_std = float(obs_heading_noise_std)
        self.obs_speed_noise_std = float(obs_speed_noise_std)

        self.enable_bad_tracking_termination = bool(enable_bad_tracking_termination)
        self.bad_tracking_min_steps = int(bad_tracking_min_steps)
        self.bad_tracking_cte_threshold = abs(float(bad_tracking_cte_threshold))
        self.bad_tracking_heading_threshold = abs(float(bad_tracking_heading_threshold))
        self.crash_penalty_value = float(crash_penalty_value)
        self.timeout_penalty_value = float(timeout_penalty_value)
        self.target_speed_smoothness_weight = float(target_speed_smoothness_weight)

        self.rng = np.random.default_rng(random_seed)

        self.average_waypoint_spacing = self._estimate_average_waypoint_spacing()

        self.env = gym.make(
            "f110_gym:f110-v0",
            map=self.map_path,
            map_ext=self.map_ext,
            num_agents=1,
        )

        self.action_space = gym.spaces.Box(
            low=np.array([-1.0], dtype=np.float32),
            high=np.array([1.0], dtype=np.float32),
            dtype=np.float32,
        )

        self.observation_space = gym.spaces.Box(
            low=np.array([
                0.0,
                self.min_speed,
                -5.0,
                -math.pi,
                0.0,
                0.0,
                0.0,
                -self.max_delta_speed_mps,
            ], dtype=np.float32),
            high=np.array([
                max(10.0, self.max_speed * 1.5),
                self.rule_max_speed_mps,
                5.0,
                math.pi,
                10.0,
                10.0,
                10.0,
                self.max_delta_speed_mps,
            ], dtype=np.float32),
            dtype=np.float32,
        )

        self.obs = None
        self.done = False
        self.step_count = 0
        self.start_centerline_idx = None
        self.previous_centerline_idx = None
        self.cumulative_progress_idx = 0.0
        self.cumulative_progress_m = 0.0
        self.previous_target_speed_mps = self.min_speed
        self.previous_delta_speed_mps = 0.0
        self.previous_correction_action = 0.0
        self.lap_completed = False
        self.crashed = False
        self.timeout = False
        self.bad_tracking_failure = False
        self.last_reset_pose = self.start_pose.copy()
        self.last_reset_random_index = -1

    def reset(self):
        """Start a new episode and clear all episode-level memory."""
        reset_pose = self._sample_reset_pose()
        self.last_reset_pose = reset_pose.copy()
        self.obs, _, self.done, _ = self.env.reset(reset_pose)

        self.step_count = 0
        self.previous_target_speed_mps = self.min_speed
        self.previous_delta_speed_mps = 0.0
        self.previous_correction_action = 0.0
        self.lap_completed = False
        self.crashed = False
        self.timeout = False
        self.bad_tracking_failure = False

        true_obs, nearest_idx = self._get_rl_observation_with_index(apply_observation_noise=False)
        self.start_centerline_idx = int(nearest_idx)
        self.previous_centerline_idx = int(nearest_idx)
        self.cumulative_progress_idx = 0.0
        self.cumulative_progress_m = 0.0

        return self._apply_observation_noise(true_obs)

    def step(self, action):
        """
        One control step. PPO chooses a physical speed residual only.
        """
        self.step_count += 1

        true_pre_obs, _ = self._get_rl_observation_with_index(apply_observation_noise=False)
        rule_speed_mps = float(true_pre_obs[1])

        correction_action = float(np.clip(action[0], -1.0, 1.0))
        delta_speed_mps = self.max_delta_speed_mps * correction_action
        requested_speed_mps = float(np.clip(rule_speed_mps + delta_speed_mps, self.min_speed, self.max_speed))
        target_speed = self._rate_limit_speed_mps(requested_speed_mps)

        car_x, car_y, car_yaw, car_speed = self._get_car_state()
        steering, _ = compute_pure_pursuit_steering(
            car_x=car_x,
            car_y=car_y,
            car_yaw=car_yaw,
            centerline=self.centerline,
            lookahead_distance=self.lookahead_distance,
            wheelbase=self.wheelbase,
            max_steer=self.max_steer,
            current_speed=car_speed,
            use_speed_dependent_lookahead=self.use_speed_dependent_lookahead,
            min_lookahead=self.min_lookahead,
            max_lookahead=self.max_lookahead,
            lookahead_speed_gain=self.lookahead_speed_gain,
        )

        gym_action = np.array([[steering, target_speed]], dtype=np.float32)
        self.obs, _, gym_done, info = self.env.step(gym_action)

        true_obs, nearest_idx = self._get_rl_observation_with_index(apply_observation_noise=False)
        rl_obs = self._apply_observation_noise(true_obs)

        # Optional early failure for cases where the car has effectively lost
        # the racing line even before the simulator reports a wall collision.
        # This is useful for high-speed training where waiting for a wall crash
        # gives a delayed and noisy learning signal.
        if (
            self.enable_bad_tracking_termination
            and self.step_count >= self.bad_tracking_min_steps
            and not self.lap_completed
            and (
                abs(float(true_obs[2])) > self.bad_tracking_cte_threshold
                or abs(float(true_obs[3])) > self.bad_tracking_heading_threshold
            )
        ):
            self.bad_tracking_failure = True
            self.crashed = True

        progress_delta_idx = compute_centerline_progress_delta(
            previous_idx=self.previous_centerline_idx,
            current_idx=nearest_idx,
            centerline_size=len(self.centerline),
        )
        forward_progress_delta_idx = max(0, progress_delta_idx)
        self.cumulative_progress_idx += float(forward_progress_delta_idx)
        self.cumulative_progress_m = self.cumulative_progress_idx * self.average_waypoint_spacing

        lap_threshold = self.lap_completion_ratio * len(self.centerline)
        if self.cumulative_progress_idx >= lap_threshold:
            self.lap_completed = True

        self.done = False
        if gym_done and not self.lap_completed:
            self.crashed = True
        if self.crashed and not self.lap_completed:
            self.done = True
        if self.lap_completed:
            self.done = True
        if self.step_count >= self.max_episode_steps:
            self.timeout = True
            self.done = True

        reward = self._compute_reward(
            true_rl_obs=true_obs,
            nearest_idx=nearest_idx,
            target_speed=target_speed,
            delta_speed_mps=delta_speed_mps,
        )

        self.previous_centerline_idx = nearest_idx
        self.previous_target_speed_mps = target_speed
        self.previous_delta_speed_mps = delta_speed_mps
        self.previous_correction_action = correction_action

        info = dict(info)
        requested_speed_index = self._speed_mps_to_index(requested_speed_mps)
        executed_speed_index = self._speed_mps_to_index(target_speed)
        rule_speed_index = self._speed_mps_to_index(rule_speed_mps)
        info["rule_speed_mps"] = rule_speed_mps
        info["rule_speed_index"] = rule_speed_index
        info["rule_min_speed_mps"] = self.rule_min_speed_mps
        info["rule_max_speed_mps"] = self.rule_max_speed_mps
        info["correction_action"] = correction_action
        info["delta_speed_mps"] = delta_speed_mps
        info["speed_index_correction"] = delta_speed_mps / max(self.max_speed - self.min_speed, 1e-6)
        info["requested_speed_mps"] = requested_speed_mps
        info["requested_speed_index"] = requested_speed_index
        info["executed_speed_index"] = executed_speed_index
        info["target_speed_mps"] = target_speed
        info["steering_rad"] = steering
        info["nearest_idx"] = int(nearest_idx)
        info["start_centerline_idx"] = int(self.start_centerline_idx) if self.start_centerline_idx is not None else -1
        info["cumulative_progress_idx"] = float(self.cumulative_progress_idx)
        info["cumulative_progress_m"] = float(self.cumulative_progress_m)
        info["lap_progress_ratio"] = float(self.cumulative_progress_idx / max(len(self.centerline), 1))
        info["curv_short_abs"] = float(true_obs[4])
        info["curv_mid_abs"] = float(true_obs[5])
        info["curv_long_abs"] = float(true_obs[6])
        info["previous_delta_speed_mps"] = float(self.previous_delta_speed_mps)
        info["previous_correction_action"] = float(self.previous_correction_action)
        info["bad_tracking_failure"] = bool(self.bad_tracking_failure)
        info["reset_random_index"] = int(self.last_reset_random_index)
        info["reset_x"] = float(self.last_reset_pose[0, 0])
        info["reset_y"] = float(self.last_reset_pose[0, 1])
        info["reset_yaw"] = float(self.last_reset_pose[0, 2])

        return rl_obs, reward, self.done, info

    def _sample_reset_pose(self) -> np.ndarray:
        """
        Sample a reset pose.

        random_start_along_centerline is the safest way to train from different
        locations: the base pose is exactly on the generated raceline. Optional
        lateral/yaw noise is clipped to small values so the car remains near the
        safe drive corridor.
        """
        if self.random_start_along_centerline:
            n = len(self.centerline)
            lo = 0 if self.random_start_min_index < 0 else max(0, self.random_start_min_index)
            hi = n - 1 if self.random_start_max_index < 0 else min(n - 1, self.random_start_max_index)
            if hi < lo:
                lo, hi = hi, lo
            idx = int(self.rng.integers(lo, hi + 1))
            p = self.centerline[idx]
            lateral = self._clipped_normal(self.start_lateral_noise_std, self.start_lateral_noise_max)
            yaw_noise = self._clipped_normal(self.start_yaw_noise_std, self.start_yaw_noise_max)
            left_normal_x = -math.sin(p.yaw)
            left_normal_y = math.cos(p.yaw)
            x = p.x + lateral * left_normal_x
            y = p.y + lateral * left_normal_y
            yaw = self._wrap_angle(p.yaw + yaw_noise)
            self.last_reset_random_index = idx
            return np.array([[x, y, yaw]], dtype=np.float32)

        # Fixed nominal start, with optional tiny global x/y/yaw noise.
        x, y, yaw = map(float, self.base_start_pose)
        dx = self._clipped_normal(self.start_xy_noise_std, self.start_xy_noise_max)
        dy = self._clipped_normal(self.start_xy_noise_std, self.start_xy_noise_max)
        yaw_noise = self._clipped_normal(self.start_yaw_noise_std, self.start_yaw_noise_max)
        self.last_reset_random_index = -1
        return np.array([[x + dx, y + dy, self._wrap_angle(yaw + yaw_noise)]], dtype=np.float32)

    def _get_car_state(self):
        car_x = float(self.obs["poses_x"][0])
        car_y = float(self.obs["poses_y"][0])
        car_yaw = float(self.obs["poses_theta"][0])
        car_speed = float(self.obs["linear_vels_x"][0])
        return car_x, car_y, car_yaw, car_speed

    def _get_rl_observation(self):
        rl_obs, _ = self._get_rl_observation_with_index(apply_observation_noise=True)
        return rl_obs

    def _get_rl_observation_with_index(self, apply_observation_noise: bool = True):
        car_x, car_y, car_yaw, car_speed = self._get_car_state()

        features, nearest_idx = get_centerline_state_features(
            car_x=car_x,
            car_y=car_y,
            car_yaw=car_yaw,
            car_speed=car_speed,
            centerline=self.centerline,
            curvature_lookahead_points=self.rule_curvature_lookahead_points,
        )

        current_speed_mps = float(features[0])
        cross_track_error = float(features[1])
        heading_error = float(features[2])
        rule_curvature_abs = float(features[3])

        curv_short_abs = get_upcoming_curvature_abs(nearest_idx, self.centerline, self.model_curvature_short_points)
        curv_mid_abs = get_upcoming_curvature_abs(nearest_idx, self.centerline, self.model_curvature_mid_points)
        curv_long_abs = get_upcoming_curvature_abs(nearest_idx, self.centerline, self.model_curvature_long_points)

        rule_speed_mps = curvature_based_speed(
            upcoming_curvature_abs=rule_curvature_abs,
            min_speed=self.rule_min_speed_mps,
            max_speed=self.rule_max_speed_mps,
            curvature_gain=self.curvature_gain,
        )

        rl_features = np.array([
            current_speed_mps,
            float(rule_speed_mps),
            cross_track_error,
            heading_error,
            float(curv_short_abs),
            float(curv_mid_abs),
            float(curv_long_abs),
            float(self.previous_delta_speed_mps),
        ], dtype=np.float32)

        if apply_observation_noise:
            rl_features = self._apply_observation_noise(rl_features)
        return rl_features, nearest_idx

    def _apply_observation_noise(self, obs: np.ndarray) -> np.ndarray:
        out = np.array(obs, dtype=np.float32, copy=True)
        if self.obs_speed_noise_std > 0.0:
            out[0] += float(self.rng.normal(0.0, self.obs_speed_noise_std))
            out[0] = float(np.clip(out[0], 0.0, max(10.0, self.max_speed * 1.5)))
        if self.obs_cte_noise_std > 0.0:
            out[2] += float(self.rng.normal(0.0, self.obs_cte_noise_std))
        if self.obs_heading_noise_std > 0.0:
            out[3] = self._wrap_angle(out[3] + float(self.rng.normal(0.0, self.obs_heading_noise_std)))
        return out

    def _compute_reward(self, true_rl_obs: np.ndarray, nearest_idx: int, target_speed: float, delta_speed_mps: float) -> float:
        """
        Simple physical-residual reward.

        Fast:
            progress reward, lap reward, early-finish bonus, small time penalty
        Trackable:
            cross-track and heading penalties, mild curvature-speed penalty
        Smooth:
            residual magnitude/change penalties and final target-speed change penalty
        Failure:
            optional bad-tracking termination and high crash penalty for high-speed learning
        """
        cross_track_error = float(true_rl_obs[2])
        heading_error = float(true_rl_obs[3])
        curv_short_abs = float(true_rl_obs[4])
        curv_mid_abs = float(true_rl_obs[5])
        curv_long_abs = float(true_rl_obs[6])

        progress_delta_idx = compute_centerline_progress_delta(
            previous_idx=self.previous_centerline_idx,
            current_idx=nearest_idx,
            centerline_size=len(self.centerline),
        )
        progress_m = progress_delta_idx * self.average_waypoint_spacing
        progress_reward = 5.0 * progress_m

        lap_reward = self.lap_bonus if self.lap_completed else 0.0
        early_finish_bonus = 0.0
        if self.lap_completed:
            remaining_ratio = max(0.0, (self.target_lap_steps - self.step_count) / max(self.target_lap_steps, 1))
            early_finish_bonus = 200.0 * remaining_ratio

        tracking_penalty = 0.8 * abs(cross_track_error) + 0.25 * abs(heading_error)

        curvature_risk_abs = max(curv_short_abs, 0.7 * curv_mid_abs, 0.5 * curv_long_abs)
        curvature_speed_penalty = 0.006 * curvature_risk_abs * target_speed * target_speed

        delta_magnitude_penalty = 0.08 * abs(delta_speed_mps)
        delta_smoothness_penalty = 0.20 * abs(delta_speed_mps - self.previous_delta_speed_mps)

        # Smooth the final speed request, not only the learned residual. This
        # complements the hard rate limiter and helps optimize the command sent
        # to the longitudinal controller/VESC. Keep it mild: racing still needs
        # real braking and acceleration.
        target_speed_smoothness_penalty = (
            self.target_speed_smoothness_weight
            * abs(target_speed - self.previous_target_speed_mps)
        )

        time_penalty = 0.02
        crash_penalty = self.crash_penalty_value if self.crashed else 0.0
        timeout_penalty = self.timeout_penalty_value if self.timeout and not self.lap_completed and not self.crashed else 0.0

        reward = (
            progress_reward
            + lap_reward
            + early_finish_bonus
            - tracking_penalty
            - curvature_speed_penalty
            - delta_magnitude_penalty
            - delta_smoothness_penalty
            - target_speed_smoothness_penalty
            - time_penalty
            - crash_penalty
            - timeout_penalty
        )
        return float(reward)

    def _speed_index_to_mps(self, speed_index: float) -> float:
        speed_index = float(np.clip(speed_index, 0.0, 1.0))
        return self.min_speed + speed_index * (self.max_speed - self.min_speed)

    def _speed_mps_to_index(self, speed_mps: float) -> float:
        speed_index = (speed_mps - self.min_speed) / (self.max_speed - self.min_speed)
        return float(np.clip(speed_index, 0.0, 1.5))

    def _rate_limit_speed_mps(self, requested_speed_mps: float) -> float:
        requested_speed_mps = float(np.clip(requested_speed_mps, self.min_speed, self.max_speed))
        if self.max_speed_delta_per_step_mps <= 0.0:
            return requested_speed_mps
        lower = self.previous_target_speed_mps - self.max_speed_delta_per_step_mps
        upper = self.previous_target_speed_mps + self.max_speed_delta_per_step_mps
        return float(np.clip(requested_speed_mps, lower, upper))

    def _estimate_average_waypoint_spacing(self) -> float:
        if len(self.centerline) < 2:
            return 0.05
        distances = []
        n = len(self.centerline)
        for i in range(n):
            p0 = self.centerline[i]
            p1 = self.centerline[(i + 1) % n]
            distances.append(math.hypot(p1.x - p0.x, p1.y - p0.y))
        return float(np.mean(distances))

    def _clipped_normal(self, std: float, max_abs: float) -> float:
        if std <= 0.0 or max_abs <= 0.0:
            return 0.0
        return float(np.clip(self.rng.normal(0.0, std), -max_abs, max_abs))

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return float(angle)