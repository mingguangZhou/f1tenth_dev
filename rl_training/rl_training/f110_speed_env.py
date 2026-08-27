import math
from typing import Tuple

import gym
import numpy as np

from rl_training.centerline_utils import (
    load_centerline_csv,
    get_centerline_state_features,
    get_upcoming_curvature_abs,
    get_upcoming_curvature_abs_by_distance,
    get_curvature_abs_section_average_by_distance,
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
        rule_speed_mps = curvature_based_speed(rule_curvature_abs, rule_min/max_speed_mps)

        physical_mps legacy mode:
            delta_speed_mps = max_delta_speed_mps * correction_action

        speed_ratio mode, recommended for highspeed-train -> reserved-apply:
            assist_ratio    = correction_action * positive/negative_assist_ratio * assist_gain
            delta_speed_mps = assist_ratio * rule_speed_mps

        requested_speed_mps = rule_speed_mps + gated_delta_speed_mps
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
        physical_mps mode:
            [current_speed_mps, rule_speed_mps, cte, heading_error,
             curv_short_abs, curv_mid_abs, curv_long_abs, previous_delta_speed_mps]

        speed_ratio mode:
            [current_speed_ratio, rule_speed_ratio, cte, heading_error,
             curv_short_abs, curv_mid_abs, curv_long_abs, previous_assist_ratio]

        The observation dimension stays 8 in both modes, but the first, second,
        and last speed-related features become dimensionless in speed_ratio mode.
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
        wheelbase_m: float = None,
        steering_max_deg: float = None,
        command_speed_min_mps: float = None,
        command_speed_max_mps: float = None,
        min_speed: float = 0.5,
        max_speed: float = 4.0,
        rule_min_speed_mps: float = None,
        rule_max_speed_mps: float = None,
        rule_curve_min_speed_mps: float = None,
        rule_straight_speed_mps: float = None,
        rule_speed_curvature_gain: float = None,
        max_speed_index_delta: float = 0.05,  # kept for backward CLI compatibility
        max_speed_delta_per_step_mps: float = 0.10,
        max_delta_speed_mps: float = 0.30,
        residual_output_mode: str = "physical_mps",
        positive_assist_ratio: float = 0.667,
        negative_assist_ratio: float = 0.50,
        assist_gain: float = 1.0,
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
        rule_speed_curvature_preview_m: float = None,
        model_curvature_short_preview_m: float = None,
        model_curvature_mid_preview_m: float = None,
        model_curvature_long_preview_m: float = None,
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
        reward_curvature_section_start_points: int = 2,
        reward_curvature_section_end_points: int = 40,
        reward_curvature_section_start_m: float = None,
        reward_curvature_section_end_m: float = None,
        curvature_speed_section_weight: float = 0.006,
        residual_smoothness_weight: float = 0.08,
        residual_free_band_mps: float = 0.8,
        residual_excess_weight: float = 0.05,
        assist_smoothness_weight: float = 0.08,
        assist_free_band: float = 0.15,
        assist_excess_weight: float = 0.05,
        enable_rl_gate: bool = False,
        rl_gate_enable_cte: float = 0.25,
        rl_gate_enable_heading: float = 0.20,
        rl_gate_disable_cte: float = 0.45,
        rl_gate_disable_heading: float = 0.35,
        rl_gate_enable_count: int = 10,
        rl_gate_disable_count: int = 3,
        rl_gate_fade_in_step: float = 0.05,
        rl_gate_fade_out_step: float = 0.10,
        random_seed: int = None,
        use_speed_dependent_lookahead: bool = True,
        use_speed_dependent_steering_lookahead: bool = None,
        fixed_steering_lookahead_m: float = None,
        min_lookahead: float = 0.6,
        max_lookahead: float = 1.6,
        lookahead_speed_gain: float = 0.25,
        steering_min_lookahead_m: float = None,
        steering_max_lookahead_m: float = None,
        steering_lookahead_speed_gain: float = None,
        min_forward_point_x_m: float = 0.05,
    ):
        super().__init__()

        # ROS path_following_v2-compatible aliases. The old argument names are
        # kept for backward compatibility, but the new YAML profiles use the
        # same terminology as path_following_v2.
        if command_speed_min_mps is not None:
            min_speed = command_speed_min_mps
        if command_speed_max_mps is not None:
            max_speed = command_speed_max_mps
        if rule_curve_min_speed_mps is not None:
            rule_min_speed_mps = rule_curve_min_speed_mps
        if rule_straight_speed_mps is not None:
            rule_max_speed_mps = rule_straight_speed_mps
        if rule_speed_curvature_gain is not None:
            curvature_gain = rule_speed_curvature_gain
        if wheelbase_m is not None:
            wheelbase = wheelbase_m
        if steering_max_deg is not None:
            max_steer = math.radians(float(steering_max_deg))
        if fixed_steering_lookahead_m is not None:
            lookahead_distance = fixed_steering_lookahead_m
        if use_speed_dependent_steering_lookahead is not None:
            use_speed_dependent_lookahead = bool(use_speed_dependent_steering_lookahead)
        if steering_min_lookahead_m is not None:
            min_lookahead = steering_min_lookahead_m
        if steering_max_lookahead_m is not None:
            max_lookahead = steering_max_lookahead_m
        if steering_lookahead_speed_gain is not None:
            lookahead_speed_gain = steering_lookahead_speed_gain

        if max_speed <= min_speed:
            raise ValueError(
                f"max_speed must be larger than min_speed. Got min_speed={min_speed}, max_speed={max_speed}."
            )
        if max_delta_speed_mps < 0.0:
            raise ValueError("max_delta_speed_mps must be non-negative")

        self.residual_output_mode = str(residual_output_mode or "physical_mps").strip().lower()
        valid_residual_modes = {"physical_mps", "speed_ratio"}
        if self.residual_output_mode not in valid_residual_modes:
            raise ValueError(
                f"residual_output_mode must be one of {sorted(valid_residual_modes)}. "
                f"Got {residual_output_mode!r}."
            )
        self.positive_assist_ratio = max(0.0, float(positive_assist_ratio))
        self.negative_assist_ratio = max(0.0, float(negative_assist_ratio))
        self.assist_gain = max(0.0, float(assist_gain))

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
        self.min_forward_point_x_m = float(min_forward_point_x_m)

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
        self.rule_speed_curvature_preview_m = (
            None if rule_speed_curvature_preview_m is None else max(0.0, float(rule_speed_curvature_preview_m))
        )
        self.model_curvature_short_preview_m = (
            None if model_curvature_short_preview_m is None else max(0.0, float(model_curvature_short_preview_m))
        )
        self.model_curvature_mid_preview_m = (
            None if model_curvature_mid_preview_m is None else max(0.0, float(model_curvature_mid_preview_m))
        )
        self.model_curvature_long_preview_m = (
            None if model_curvature_long_preview_m is None else max(0.0, float(model_curvature_long_preview_m))
        )

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
        self.reward_curvature_section_start_points = max(0, int(reward_curvature_section_start_points))
        self.reward_curvature_section_end_points = max(
            self.reward_curvature_section_start_points,
            int(reward_curvature_section_end_points),
        )
        self.reward_curvature_section_start_m = (
            None if reward_curvature_section_start_m is None else max(0.0, float(reward_curvature_section_start_m))
        )
        self.reward_curvature_section_end_m = (
            None if reward_curvature_section_end_m is None else max(
                self.reward_curvature_section_start_m if self.reward_curvature_section_start_m is not None else 0.0,
                float(reward_curvature_section_end_m),
            )
        )
        self.curvature_speed_section_weight = float(curvature_speed_section_weight)
        self.residual_smoothness_weight = max(0.0, float(residual_smoothness_weight))
        self.residual_free_band_mps = max(0.0, float(residual_free_band_mps))
        self.residual_excess_weight = max(0.0, float(residual_excess_weight))
        self.assist_smoothness_weight = max(0.0, float(assist_smoothness_weight))
        self.assist_free_band = max(0.0, float(assist_free_band))
        self.assist_excess_weight = max(0.0, float(assist_excess_weight))
        self.last_reward_terms = {}
        self.last_observation_meta = {}

        # Optional runtime-style safety gate for the learned residual.
        # When enabled, the PPO residual is used only inside a near-raceline
        # operating domain. Outside that domain the residual fades to zero and
        # the car falls back to the curvature-rule speed. Hysteresis and
        # counters prevent mode flicker during recoverable transient errors.
        self.enable_rl_gate = bool(enable_rl_gate)
        self.rl_gate_enable_cte = abs(float(rl_gate_enable_cte))
        self.rl_gate_enable_heading = abs(float(rl_gate_enable_heading))
        self.rl_gate_disable_cte = abs(float(rl_gate_disable_cte))
        self.rl_gate_disable_heading = abs(float(rl_gate_disable_heading))
        self.rl_gate_enable_count = max(1, int(rl_gate_enable_count))
        self.rl_gate_disable_count = max(1, int(rl_gate_disable_count))
        self.rl_gate_fade_in_step = float(np.clip(rl_gate_fade_in_step, 0.0, 1.0))
        self.rl_gate_fade_out_step = float(np.clip(rl_gate_fade_out_step, 0.0, 1.0))
        self.rl_gate_enabled = not self.enable_rl_gate
        self.rl_gate_scale = 1.0 if not self.enable_rl_gate else 0.0
        self.rl_gate_good_count = 0
        self.rl_gate_bad_count = 0

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

        if self.residual_output_mode == "speed_ratio":
            max_assist_feature = max(1.0, self.assist_gain * max(self.positive_assist_ratio, self.negative_assist_ratio))
            obs_low = np.array([
                0.0,
                0.0,
                -5.0,
                -math.pi,
                0.0,
                0.0,
                0.0,
                -max_assist_feature,
            ], dtype=np.float32)
            obs_high = np.array([
                1.5,
                1.5,
                5.0,
                math.pi,
                10.0,
                10.0,
                10.0,
                max_assist_feature,
            ], dtype=np.float32)
        else:
            obs_low = np.array([
                0.0,
                self.min_speed,
                -5.0,
                -math.pi,
                0.0,
                0.0,
                0.0,
                -self.max_delta_speed_mps,
            ], dtype=np.float32)
            obs_high = np.array([
                max(10.0, self.max_speed * 1.5),
                self.rule_max_speed_mps,
                5.0,
                math.pi,
                10.0,
                10.0,
                10.0,
                self.max_delta_speed_mps,
            ], dtype=np.float32)

        self.observation_space = gym.spaces.Box(
            low=obs_low,
            high=obs_high,
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
        self.previous_assist_ratio = 0.0
        self.previous_correction_action = 0.0
        self._reset_rl_gate_state()
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
        self.previous_assist_ratio = 0.0
        self.previous_correction_action = 0.0
        self._reset_rl_gate_state()
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
        rule_speed_mps = float(self.last_observation_meta.get("rule_speed_mps", true_pre_obs[1]))

        correction_action = float(np.clip(action[0], -1.0, 1.0))
        raw_delta_speed_mps, raw_assist_ratio = self._action_to_raw_residual(
            correction_action=correction_action,
            rule_speed_mps=rule_speed_mps,
        )

        gate_enabled, gate_scale = self._update_rl_gate(
            cross_track_error=float(true_pre_obs[2]),
            heading_error=float(true_pre_obs[3]),
        )

        # The model action is still visible in info/debug logs, but only the
        # gated residual is sent into the speed command. If the gate is disabled
        # or fading out, this smoothly falls back to rule_speed_mps.
        delta_speed_mps = gate_scale * raw_delta_speed_mps
        assist_ratio = gate_scale * raw_assist_ratio
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
            # path_following_v2 uses the final commanded speed for speed-dependent
            # steering lookahead, not the current measured vehicle speed.
            current_speed=target_speed,
            use_speed_dependent_lookahead=self.use_speed_dependent_lookahead,
            min_lookahead=self.min_lookahead,
            max_lookahead=self.max_lookahead,
            lookahead_speed_gain=self.lookahead_speed_gain,
            min_forward_point_x_m=self.min_forward_point_x_m,
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
            assist_ratio=assist_ratio,
        )

        self.previous_centerline_idx = nearest_idx
        self.previous_target_speed_mps = target_speed
        self.previous_delta_speed_mps = delta_speed_mps
        self.previous_assist_ratio = assist_ratio
        self.previous_correction_action = correction_action

        info = dict(info)
        requested_speed_index = self._speed_mps_to_index(requested_speed_mps)
        executed_speed_index = self._speed_mps_to_index(target_speed)
        rule_speed_index = self._speed_mps_to_index(rule_speed_mps)
        info["rule_speed_mps"] = rule_speed_mps
        info["rule_speed_index"] = rule_speed_index
        info["rule_min_speed_mps"] = self.rule_min_speed_mps
        info["rule_max_speed_mps"] = self.rule_max_speed_mps
        info["residual_output_mode"] = self.residual_output_mode
        info["correction_action"] = correction_action
        info["raw_delta_speed_mps"] = raw_delta_speed_mps
        info["raw_assist_ratio"] = raw_assist_ratio
        info["assist_ratio"] = assist_ratio
        info["assist_gain"] = self.assist_gain
        info["positive_assist_ratio"] = self.positive_assist_ratio
        info["negative_assist_ratio"] = self.negative_assist_ratio
        info["rl_gate_enabled"] = bool(gate_enabled)
        info["rl_gate_scale"] = float(gate_scale)
        info["rl_gate_good_count"] = int(self.rl_gate_good_count)
        info["rl_gate_bad_count"] = int(self.rl_gate_bad_count)
        info["delta_speed_mps"] = delta_speed_mps
        info["speed_index_correction"] = delta_speed_mps / max(self.max_speed - self.min_speed, 1e-6)
        info["requested_speed_mps"] = requested_speed_mps
        info["requested_speed_index"] = requested_speed_index
        info["executed_speed_index"] = executed_speed_index
        info["target_speed_mps"] = target_speed
        info["current_speed_mps"] = float(self.last_observation_meta.get("current_speed_mps", float("nan")))
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
        info["previous_assist_ratio"] = float(self.previous_assist_ratio)
        info["previous_correction_action"] = float(self.previous_correction_action)
        info["bad_tracking_failure"] = bool(self.bad_tracking_failure)
        info["reset_random_index"] = int(self.last_reset_random_index)
        info["reset_x"] = float(self.last_reset_pose[0, 0])
        info["reset_y"] = float(self.last_reset_pose[0, 1])
        info["reset_yaw"] = float(self.last_reset_pose[0, 2])
        for key, value in self.last_reward_terms.items():
            info[key] = value

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

        if self.rule_speed_curvature_preview_m is not None:
            rule_curvature_abs = get_upcoming_curvature_abs_by_distance(
                nearest_idx, self.centerline, self.rule_speed_curvature_preview_m
            )
        else:
            rule_curvature_abs = float(features[3])

        curv_short_abs = self._get_preview_curvature_abs(
            nearest_idx, self.model_curvature_short_preview_m, self.model_curvature_short_points
        )
        curv_mid_abs = self._get_preview_curvature_abs(
            nearest_idx, self.model_curvature_mid_preview_m, self.model_curvature_mid_points
        )
        curv_long_abs = self._get_preview_curvature_abs(
            nearest_idx, self.model_curvature_long_preview_m, self.model_curvature_long_points
        )

        rule_speed_mps = curvature_based_speed(
            upcoming_curvature_abs=rule_curvature_abs,
            min_speed=self.rule_min_speed_mps,
            max_speed=self.rule_max_speed_mps,
            curvature_gain=self.curvature_gain,
        )

        if self.residual_output_mode == "speed_ratio":
            previous_assist_feature = float(self.previous_assist_ratio)
            rl_features = np.array([
                self._normalize_command_speed(current_speed_mps),
                self._normalize_rule_speed(rule_speed_mps),
                cross_track_error,
                heading_error,
                float(curv_short_abs),
                float(curv_mid_abs),
                float(curv_long_abs),
                previous_assist_feature,
            ], dtype=np.float32)
        else:
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

        self.last_observation_meta = {
            "current_speed_mps": float(current_speed_mps),
            "rule_speed_mps": float(rule_speed_mps),
            "cross_track_error": float(cross_track_error),
            "heading_error": float(heading_error),
            "nearest_idx": int(nearest_idx),
        }

        if apply_observation_noise:
            rl_features = self._apply_observation_noise(rl_features)
        return rl_features, nearest_idx

    def _apply_observation_noise(self, obs: np.ndarray) -> np.ndarray:
        out = np.array(obs, dtype=np.float32, copy=True)
        if self.obs_speed_noise_std > 0.0:
            out[0] += float(self.rng.normal(0.0, self.obs_speed_noise_std))
            if self.residual_output_mode == "speed_ratio":
                out[0] = float(np.clip(out[0], 0.0, 1.5))
            else:
                out[0] = float(np.clip(out[0], 0.0, max(10.0, self.max_speed * 1.5)))
        if self.obs_cte_noise_std > 0.0:
            out[2] += float(self.rng.normal(0.0, self.obs_cte_noise_std))
        if self.obs_heading_noise_std > 0.0:
            out[3] = self._wrap_angle(out[3] + float(self.rng.normal(0.0, self.obs_heading_noise_std)))
        return out

    def _compute_reward(
        self,
        true_rl_obs: np.ndarray,
        nearest_idx: int,
        target_speed: float,
        delta_speed_mps: float,
        assist_ratio: float,
    ) -> float:
        """
        Behavior-focused physical-residual reward.

        The reward mainly judges the resulting driving behavior, while keeping
        two mild residual regularizers to discourage oscillatory or excessive
        residual commands:
          - make forward progress and finish the lap quickly,
          - stay close/aligned to the raceline,
          - avoid high speed through a curved lookahead section,
          - keep the final target-speed command smooth,
          - avoid crash/bad-tracking/timeout failures.

        The policy observation still contains short/mid/long curvature preview.
        The reward separately uses an average absolute curvature over a fixed
        metre-based section ahead of the vehicle when configured. Legacy
        point-count section parameters remain as a fallback.
        """
        cross_track_error = float(true_rl_obs[2])
        heading_error = float(true_rl_obs[3])

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

        curvature_section_abs = self._get_curvature_section_average_abs(
            nearest_idx=nearest_idx,
            start_offset_points=self.reward_curvature_section_start_points,
            end_offset_points=self.reward_curvature_section_end_points,
        )
        curvature_speed_section_penalty = (
            self.curvature_speed_section_weight
            * curvature_section_abs
            * target_speed
            * target_speed
        )

        # Smooth the final speed request, not the learned residual. This is the
        # clearest indicator for longitudinal command smoothness.
        target_speed_smoothness_penalty = (
            self.target_speed_smoothness_weight
            * abs(target_speed - self.previous_target_speed_mps)
        )

        # Mild residual/assist regularization. In physical_mps mode this acts on
        # physical residual m/s. In speed_ratio mode it acts on the dimensionless
        # assist ratio so that a trend learned in the high-speed profile can be
        # applied to a reserved profile without carrying over absolute m/s costs.
        if self.residual_output_mode == "speed_ratio":
            residual_smoothness_penalty = 0.0
            residual_excess = 0.0
            residual_excess_penalty = 0.0
            assist_smoothness_penalty = (
                self.assist_smoothness_weight
                * abs(float(assist_ratio) - self.previous_assist_ratio)
            )
            assist_excess = max(0.0, abs(float(assist_ratio)) - self.assist_free_band)
            assist_excess_penalty = self.assist_excess_weight * assist_excess * assist_excess
        else:
            residual_smoothness_penalty = (
                self.residual_smoothness_weight
                * abs(delta_speed_mps - self.previous_delta_speed_mps)
            )
            residual_excess = max(0.0, abs(delta_speed_mps) - self.residual_free_band_mps)
            residual_excess_penalty = self.residual_excess_weight * residual_excess * residual_excess
            assist_smoothness_penalty = 0.0
            assist_excess = 0.0
            assist_excess_penalty = 0.0

        time_penalty = 0.02
        crash_penalty = self.crash_penalty_value if self.crashed else 0.0
        timeout_penalty = self.timeout_penalty_value if self.timeout and not self.lap_completed and not self.crashed else 0.0

        reward = (
            progress_reward
            + lap_reward
            + early_finish_bonus
            - tracking_penalty
            - curvature_speed_section_penalty
            - target_speed_smoothness_penalty
            - residual_smoothness_penalty
            - residual_excess_penalty
            - assist_smoothness_penalty
            - assist_excess_penalty
            - time_penalty
            - crash_penalty
            - timeout_penalty
        )

        self.last_reward_terms = {
            "reward_progress": float(progress_reward),
            "reward_lap": float(lap_reward),
            "reward_early_finish": float(early_finish_bonus),
            "penalty_tracking": float(tracking_penalty),
            "penalty_curvature_speed_section": float(curvature_speed_section_penalty),
            "penalty_target_speed_smoothness": float(target_speed_smoothness_penalty),
            "penalty_residual_smoothness": float(residual_smoothness_penalty),
            "penalty_residual_excess": float(residual_excess_penalty),
            "residual_excess_mps": float(residual_excess),
            "penalty_assist_smoothness": float(assist_smoothness_penalty),
            "penalty_assist_excess": float(assist_excess_penalty),
            "assist_excess_ratio": float(assist_excess),
            "reward_assist_ratio": float(assist_ratio),
            "penalty_time": float(time_penalty),
            "penalty_crash": float(crash_penalty),
            "penalty_timeout": float(timeout_penalty),
            "reward_total": float(reward),
            "reward_curvature_section_abs": float(curvature_section_abs),
        }
        return float(reward)


    def _normalize_command_speed(self, speed_mps: float) -> float:
        """Normalize physical command speed to the current profile envelope."""
        denom = max(self.max_speed - self.min_speed, 1e-6)
        return float(np.clip((float(speed_mps) - self.min_speed) / denom, -0.5, 1.5))

    def _normalize_rule_speed(self, rule_speed_mps: float) -> float:
        """Normalize rule speed inside the rule profile envelope."""
        denom = max(self.rule_max_speed_mps - self.rule_min_speed_mps, 1e-6)
        return float(np.clip((float(rule_speed_mps) - self.rule_min_speed_mps) / denom, -0.5, 1.5))

    def _action_to_raw_residual(self, correction_action: float, rule_speed_mps: float):
        """Convert PPO action into a raw residual before gate scaling.

        physical_mps mode preserves the legacy interpretation: action scales a
        fixed m/s residual authority.

        speed_ratio mode interprets action as a dimensionless assist trend. The
        resulting m/s residual is computed relative to the current rule speed,
        which lets the same policy be trained with a high-speed profile and then
        applied to a reserved profile with a different rule-speed scale.
        """
        action = float(np.clip(correction_action, -1.0, 1.0))
        rule_speed = max(float(rule_speed_mps), 1e-6)
        if self.residual_output_mode == "speed_ratio":
            if action >= 0.0:
                raw_assist_ratio = action * self.positive_assist_ratio * self.assist_gain
            else:
                raw_assist_ratio = action * self.negative_assist_ratio * self.assist_gain
            raw_delta_speed_mps = raw_assist_ratio * rule_speed
            return float(raw_delta_speed_mps), float(raw_assist_ratio)

        raw_delta_speed_mps = self.max_delta_speed_mps * action
        raw_assist_ratio = raw_delta_speed_mps / rule_speed
        return float(raw_delta_speed_mps), float(raw_assist_ratio)

    def _get_preview_curvature_abs(self, nearest_idx: int, preview_m, fallback_points: int) -> float:
        """Get max abs curvature using metre preview when available, else point count."""
        if preview_m is not None:
            return get_upcoming_curvature_abs_by_distance(
                nearest_idx, self.centerline, float(preview_m)
            )
        return get_upcoming_curvature_abs(nearest_idx, self.centerline, int(fallback_points))

    def _get_curvature_section_average_abs(self, nearest_idx: int, start_offset_points: int, end_offset_points: int) -> float:
        """Average absolute curvature over a forward section of the closed raceline."""
        if self.reward_curvature_section_start_m is not None or self.reward_curvature_section_end_m is not None:
            start_m = 0.0 if self.reward_curvature_section_start_m is None else self.reward_curvature_section_start_m
            end_m = start_m if self.reward_curvature_section_end_m is None else self.reward_curvature_section_end_m
            return get_curvature_abs_section_average_by_distance(
                nearest_idx=nearest_idx,
                centerline=self.centerline,
                start_distance_m=start_m,
                end_distance_m=end_m,
            )

        n = len(self.centerline)
        if n <= 0:
            return 0.0
        start = max(0, int(start_offset_points))
        end = max(start, int(end_offset_points))
        values = []
        for offset in range(start, end + 1):
            p = self.centerline[(int(nearest_idx) + offset) % n]
            values.append(abs(float(p.curvature_abs)))
        if not values:
            return 0.0
        return float(np.mean(values))

    def _reset_rl_gate_state(self) -> None:
        """Reset the residual gate at the beginning of an episode."""
        if self.enable_rl_gate:
            self.rl_gate_enabled = False
            self.rl_gate_scale = 0.0
        else:
            self.rl_gate_enabled = True
            self.rl_gate_scale = 1.0
        self.rl_gate_good_count = 0
        self.rl_gate_bad_count = 0

    def _update_rl_gate(self, cross_track_error: float, heading_error: float):
        """
        Runtime-style hysteresis gate for the learned speed residual.

        OFF -> ON requires a clearly good tracking state for several cycles.
        ON -> OFF requires a clearly bad tracking state for several cycles.
        The returned scale fades the residual in/out to avoid speed-command jumps.
        """
        if not self.enable_rl_gate:
            self.rl_gate_enabled = True
            self.rl_gate_scale = 1.0
            self.rl_gate_good_count = 0
            self.rl_gate_bad_count = 0
            return self.rl_gate_enabled, self.rl_gate_scale

        abs_cte = abs(float(cross_track_error))
        abs_heading = abs(float(heading_error))

        good = (
            abs_cte < self.rl_gate_enable_cte
            and abs_heading < self.rl_gate_enable_heading
        )
        bad = (
            abs_cte > self.rl_gate_disable_cte
            or abs_heading > self.rl_gate_disable_heading
        )

        if good:
            self.rl_gate_good_count += 1
        else:
            self.rl_gate_good_count = 0

        if bad:
            self.rl_gate_bad_count += 1
        else:
            self.rl_gate_bad_count = 0

        if (not self.rl_gate_enabled) and self.rl_gate_good_count >= self.rl_gate_enable_count:
            self.rl_gate_enabled = True

        if self.rl_gate_enabled and self.rl_gate_bad_count >= self.rl_gate_disable_count:
            self.rl_gate_enabled = False

        if self.rl_gate_enabled:
            self.rl_gate_scale = min(1.0, self.rl_gate_scale + self.rl_gate_fade_in_step)
        else:
            self.rl_gate_scale = max(0.0, self.rl_gate_scale - self.rl_gate_fade_out_step)

        return self.rl_gate_enabled, self.rl_gate_scale

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