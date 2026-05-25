import math
from typing import List, Tuple

import gym
import numpy as np

from rl_training.centerline_utils import (
    CenterlinePoint,
    load_centerline_csv,
    get_centerline_state_features,
    compute_centerline_progress_delta,
)
from rl_training.pure_pursuit import compute_pure_pursuit_steering


class F110RacingEnv(gym.Env):
    """
    Minimal RL environment for combined longitudinal + lateral planning.

    Observation:
        [
            current_speed,
            cross_track_error,
            heading_error,
            upcoming_curvature_abs,
        ]

    Action:
        [
            normalized_speed_request,       # [0, 1]
            normalized_lateral_request,     # [-1, 1]
        ]

    Control layer:
        1. Map normalized_speed_request to target_speed.
        2. Map normalized_lateral_request to lateral_offset.
        3. Apply simple per-step rate limits.
        4. Shift the centerline by lateral_offset.
        5. Use pure pursuit to generate steering.
        6. Send [steering, target_speed] to F1TENTH Gym.

    This deliberately keeps RL at the planning-request level, while pure pursuit
    remains the low-level stabilizing controller.
    """

    metadata = {"render.modes": ["human"]}

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
        max_lateral_offset: float = 0.35,
        max_speed_delta_per_step: float = 0.15,
        max_lateral_delta_per_step: float = 0.03,
        max_episode_steps: int = 3000,
        lap_completion_ratio: float = 0.95,
        lap_bonus: float = 500.0,
        use_speed_dependent_lookahead: bool = True,
        min_lookahead: float = 0.6,
        max_lookahead: float = 1.6,
        lookahead_speed_gain: float = 0.25,
        curvature_lookahead_points: int = 20,
        progress_reward_gain: float = 2.0,
        speed_bonus_gain: float = 0.05,
        lateral_error_gain: float = 0.30,
        heading_error_gain: float = 0.15,
        speed_smoothness_gain: float = 0.05,
        lateral_smoothness_gain: float = 0.10,
        steering_smoothness_gain: float = 0.05,
        lateral_offset_gain: float = 0.05,
        curve_speed_gain: float = 0.005,
        crash_penalty_value: float = 100.0,
    ):
        super().__init__()

        self.map_path = map_path
        self.map_ext = map_ext
        self.centerline = load_centerline_csv(centerline_csv)
        self.start_pose = np.array([start_pose], dtype=np.float32)

        self.lookahead_distance = lookahead_distance
        self.wheelbase = wheelbase
        self.max_steer = max_steer
        self.use_speed_dependent_lookahead = use_speed_dependent_lookahead
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.lookahead_speed_gain = lookahead_speed_gain
        self.curvature_lookahead_points = curvature_lookahead_points

        self.min_speed = min_speed
        self.max_speed = max_speed
        self.max_lateral_offset = max_lateral_offset
        self.max_speed_delta_per_step = max_speed_delta_per_step
        self.max_lateral_delta_per_step = max_lateral_delta_per_step

        self.max_episode_steps = max_episode_steps
        self.lap_completion_ratio = lap_completion_ratio
        self.lap_bonus = lap_bonus

        # Reward weights. Keep them explicit and easy to tune.
        self.progress_reward_gain = progress_reward_gain
        self.speed_bonus_gain = speed_bonus_gain
        self.lateral_error_gain = lateral_error_gain
        self.heading_error_gain = heading_error_gain
        self.speed_smoothness_gain = speed_smoothness_gain
        self.lateral_smoothness_gain = lateral_smoothness_gain
        self.steering_smoothness_gain = steering_smoothness_gain
        self.lateral_offset_gain = lateral_offset_gain
        self.curve_speed_gain = curve_speed_gain
        self.crash_penalty_value = crash_penalty_value

        self.env = gym.make(
            "f110_gym:f110-v0",
            map=self.map_path,
            map_ext=self.map_ext,
            num_agents=1,
        )

        # Normalized action space. This makes the trained model less tied to one
        # specific physical speed/lateral range.
        self.action_space = gym.spaces.Box(
            low=np.array([0.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32,
        )

        self.observation_space = gym.spaces.Box(
            low=np.array([0.0, -5.0, -math.pi, 0.0], dtype=np.float32),
            high=np.array([20.0, 5.0, math.pi, 10.0], dtype=np.float32),
            dtype=np.float32,
        )

        self.obs = None
        self.done = False
        self.step_count = 0

        self.previous_centerline_idx = None
        self.max_progress_idx = 0

        self.prev_target_speed = None
        self.prev_lateral_offset = None
        self.prev_steering = None

        self.last_raw_action = np.zeros(2, dtype=np.float32)
        self.last_target_speed = min_speed
        self.last_lateral_offset = 0.0
        self.last_steering = 0.0

        self.lap_completed = False
        self.crashed = False
        self.timeout = False

    def reset(self):
        self.obs, _, self.done, _ = self.env.reset(self.start_pose)

        self.step_count = 0
        self.previous_centerline_idx = None
        self.max_progress_idx = 0

        self.prev_target_speed = None
        self.prev_lateral_offset = None
        self.prev_steering = None

        self.last_raw_action = np.zeros(2, dtype=np.float32)
        self.last_target_speed = self.min_speed
        self.last_lateral_offset = 0.0
        self.last_steering = 0.0

        self.lap_completed = False
        self.crashed = False
        self.timeout = False

        return self._get_rl_observation()

    def step(self, action):
        self.step_count += 1

        raw_action = np.asarray(action, dtype=np.float32).reshape(-1)
        if raw_action.shape[0] != 2:
            raise ValueError(f"Expected action with shape (2,), got {raw_action.shape}")

        self.last_raw_action = np.array(
            [
                np.clip(raw_action[0], 0.0, 1.0),
                np.clip(raw_action[1], -1.0, 1.0),
            ],
            dtype=np.float32,
        )

        requested_speed, requested_lateral_offset = self._map_action_to_requests(
            self.last_raw_action
        )

        target_speed = self._rate_limit(
            requested_speed,
            self.prev_target_speed,
            self.max_speed_delta_per_step,
        )
        lateral_offset = self._rate_limit(
            requested_lateral_offset,
            self.prev_lateral_offset,
            self.max_lateral_delta_per_step,
        )

        car_x, car_y, car_yaw, car_speed = self._get_car_state()

        shifted_centerline = shift_centerline_laterally(
            centerline=self.centerline,
            lateral_offset=lateral_offset,
        )

        steering, _ = compute_pure_pursuit_steering(
            car_x=car_x,
            car_y=car_y,
            car_yaw=car_yaw,
            centerline=shifted_centerline,
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

        rl_obs, nearest_idx = self._get_rl_observation_with_index()

        self.max_progress_idx = max(self.max_progress_idx, nearest_idx)
        lap_threshold = int(self.lap_completion_ratio * len(self.centerline))
        if self.max_progress_idx >= lap_threshold:
            self.lap_completed = True

        self.done = False
        if gym_done and not self.lap_completed:
            self.crashed = True
            self.done = True
        if self.lap_completed:
            self.done = True
        if self.step_count >= self.max_episode_steps:
            self.timeout = True
            self.done = True

        reward = self._compute_reward(
            rl_obs=rl_obs,
            nearest_idx=nearest_idx,
            target_speed=target_speed,
            lateral_offset=lateral_offset,
            steering=steering,
        )

        self.previous_centerline_idx = nearest_idx
        self.prev_target_speed = target_speed
        self.prev_lateral_offset = lateral_offset
        self.prev_steering = steering

        self.last_target_speed = target_speed
        self.last_lateral_offset = lateral_offset
        self.last_steering = steering

        return rl_obs, reward, self.done, info

    def _map_action_to_requests(self, action: np.ndarray) -> Tuple[float, float]:
        normalized_speed = float(np.clip(action[0], 0.0, 1.0))
        normalized_lateral = float(np.clip(action[1], -1.0, 1.0))

        target_speed = self.min_speed + normalized_speed * (self.max_speed - self.min_speed)
        lateral_offset = normalized_lateral * self.max_lateral_offset

        return float(target_speed), float(lateral_offset)

    @staticmethod
    def _rate_limit(requested_value, previous_value, max_delta):
        if previous_value is None:
            return float(requested_value)
        delta = requested_value - previous_value
        delta = float(np.clip(delta, -max_delta, max_delta))
        return float(previous_value + delta)

    def _get_car_state(self):
        car_x = float(self.obs["poses_x"][0])
        car_y = float(self.obs["poses_y"][0])
        car_yaw = float(self.obs["poses_theta"][0])
        car_speed = float(self.obs["linear_vels_x"][0])
        return car_x, car_y, car_yaw, car_speed

    def _get_rl_observation(self):
        rl_obs, _ = self._get_rl_observation_with_index()
        return rl_obs

    def _get_rl_observation_with_index(self):
        car_x, car_y, car_yaw, car_speed = self._get_car_state()

        features, nearest_idx = get_centerline_state_features(
            car_x=car_x,
            car_y=car_y,
            car_yaw=car_yaw,
            car_speed=car_speed,
            centerline=self.centerline,
            curvature_lookahead_points=self.curvature_lookahead_points,
        )

        return np.array(features, dtype=np.float32), nearest_idx

    def _compute_reward(
        self,
        rl_obs: np.ndarray,
        nearest_idx: int,
        target_speed: float,
        lateral_offset: float,
        steering: float,
    ) -> float:
        actual_speed = float(rl_obs[0])
        cross_track_error = float(rl_obs[1])
        heading_error = float(rl_obs[2])
        upcoming_curvature_abs = float(rl_obs[3])

        progress_delta = compute_centerline_progress_delta(
            previous_idx=self.previous_centerline_idx,
            current_idx=nearest_idx,
            centerline_size=len(self.centerline),
        )

        progress_reward = self.progress_reward_gain * progress_delta
        speed_bonus = self.speed_bonus_gain * actual_speed

        lateral_error_penalty = self.lateral_error_gain * abs(cross_track_error)
        heading_error_penalty = self.heading_error_gain * abs(heading_error)

        if self.prev_target_speed is None:
            speed_smoothness_penalty = 0.0
        else:
            speed_smoothness_penalty = self.speed_smoothness_gain * abs(
                target_speed - self.prev_target_speed
            )

        if self.prev_lateral_offset is None:
            lateral_smoothness_penalty = 0.0
        else:
            lateral_smoothness_penalty = self.lateral_smoothness_gain * abs(
                lateral_offset - self.prev_lateral_offset
            )

        if self.prev_steering is None:
            steering_smoothness_penalty = 0.0
        else:
            steering_smoothness_penalty = self.steering_smoothness_gain * abs(
                steering - self.prev_steering
            )

        lateral_offset_penalty = self.lateral_offset_gain * abs(lateral_offset)

        # Keep this weak. It discourages reckless high speed in high curvature,
        # but does not dominate progress reward.
        curve_speed_penalty = (
            self.curve_speed_gain
            * upcoming_curvature_abs
            * actual_speed
            * actual_speed
        )

        crash_penalty = self.crash_penalty_value if self.crashed else 0.0
        lap_reward = self.lap_bonus if self.lap_completed else 0.0

        reward = (
            progress_reward
            + speed_bonus
            + lap_reward
            - lateral_error_penalty
            - heading_error_penalty
            - speed_smoothness_penalty
            - lateral_smoothness_penalty
            - steering_smoothness_penalty
            - lateral_offset_penalty
            - curve_speed_penalty
            - crash_penalty
        )

        return float(reward)


def shift_centerline_laterally(
    centerline: List[CenterlinePoint],
    lateral_offset: float,
) -> List[CenterlinePoint]:
    """
    Shift every centerline point along its local left normal.

    positive lateral_offset: left of centerline driving direction
    negative lateral_offset: right of centerline driving direction

    This is intentionally simple for Phase 1. Later we can shift only a local
    horizon segment, clamp by track width, or smooth offset along the horizon.
    """
    shifted = []

    for p in centerline:
        normal_x = -math.sin(p.yaw)
        normal_y = math.cos(p.yaw)

        shifted.append(
            CenterlinePoint(
                index=p.index,
                x=p.x + lateral_offset * normal_x,
                y=p.y + lateral_offset * normal_y,
                yaw=p.yaw,
                curvature=p.curvature,
                curvature_abs=p.curvature_abs,
            )
        )

    return shifted
