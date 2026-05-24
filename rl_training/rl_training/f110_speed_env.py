import math
from typing import Tuple

import gym
import numpy as np

from rl_training.centerline_utils import (
    load_centerline_csv,
    get_centerline_state_features,
    compute_centerline_progress_delta,
)
from rl_training.pure_pursuit import compute_pure_pursuit_steering


class F110SpeedEnv(gym.Env):
    """
    Minimal RL environment for speed-only learning.

    Action:
        [target_speed]

    Steering:
        Rule-based pure pursuit.

    Observation:
        [
            current_speed,
            cross_track_error,
            heading_error,
            upcoming_curvature_abs,
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
        max_episode_steps: int = 2000,
        lap_completion_ratio: float = 0.95,
        lap_bonus: float = 500.0,
        use_speed_dependent_lookahead: bool = True,
        min_lookahead: float = 0.6,
        max_lookahead: float = 1.6,
        lookahead_speed_gain: float = 0.25,
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

        # Same speed range will be used by both rule-based baseline and RL agent.
        self.min_speed = min_speed
        self.max_speed = max_speed

        self.max_episode_steps = max_episode_steps
        self.lap_completion_ratio = lap_completion_ratio
        self.lap_bonus = lap_bonus

        self.env = gym.make(
            "f110_gym:f110-v0",
            map=self.map_path,
            map_ext=self.map_ext,
            num_agents=1,
        )

        # RL action: one continuous value = target speed.
        self.action_space = gym.spaces.Box(
            low=np.array([self.min_speed], dtype=np.float32),
            high=np.array([self.max_speed], dtype=np.float32),
            dtype=np.float32,
        )

        # RL observation:
        # [current_speed, cross_track_error, heading_error, upcoming_curvature_abs]
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

        self.lap_completed = False
        self.crashed = False
        self.timeout = False

    def reset(self):
        """
        Start a new episode and clear all episode-level memory.
        """
        self.obs, _, self.done, _ = self.env.reset(self.start_pose)

        self.step_count = 0
        self.previous_centerline_idx = None
        self.max_progress_idx = 0

        self.lap_completed = False
        self.crashed = False
        self.timeout = False

        return self._get_rl_observation()

    def step(self, action):
        """
        One control step.

        RL chooses only target speed.
        Pure pursuit computes steering.
        F1TENTH Gym receives [steering, speed].
        """
        self.step_count += 1

        target_speed = float(np.clip(action[0], self.min_speed, self.max_speed))

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

        rl_obs, nearest_idx = self._get_rl_observation_with_index()

        # Update lap progress.
        self.max_progress_idx = max(self.max_progress_idx, nearest_idx)
        lap_threshold = int(self.lap_completion_ratio * len(self.centerline))

        if self.max_progress_idx >= lap_threshold:
            self.lap_completed = True

        # Distinguish episode termination reasons.
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
        )

        self.previous_centerline_idx = nearest_idx

        return rl_obs, reward, self.done, info

    def _get_car_state(self):
        """
        Extract ego vehicle state from f1tenth_gym observation.
        """
        car_x = float(self.obs["poses_x"][0])
        car_y = float(self.obs["poses_y"][0])
        car_yaw = float(self.obs["poses_theta"][0])
        car_speed = float(self.obs["linear_vels_x"][0])

        return car_x, car_y, car_yaw, car_speed

    def _get_rl_observation(self):
        """
        Return only the RL observation.
        """
        rl_obs, _ = self._get_rl_observation_with_index()
        return rl_obs

    def _get_rl_observation_with_index(self):
        """
        Return RL observation plus nearest centerline index.

        The index is not given to the RL agent directly.
        It is used internally for progress/lap reward.
        """
        car_x, car_y, car_yaw, car_speed = self._get_car_state()

        features, nearest_idx = get_centerline_state_features(
            car_x=car_x,
            car_y=car_y,
            car_yaw=car_yaw,
            car_speed=car_speed,
            centerline=self.centerline,
            curvature_lookahead_points=20,
        )

        return np.array(features, dtype=np.float32), nearest_idx

    def _compute_reward(
        self,
        rl_obs: np.ndarray,
        nearest_idx: int,
        target_speed: float,
    ) -> float:
        """
        Reward for speed-only RL.

        Positive:
            - forward progress along centerline
            - lap completion bonus

        Negative:
            - crash
            - large lateral error
            - large heading error
            - excessive speed in curves
        """
        cross_track_error = float(rl_obs[1])
        heading_error = float(rl_obs[2])
        upcoming_curvature_abs = float(rl_obs[3])

        progress_delta = compute_centerline_progress_delta(
            previous_idx=self.previous_centerline_idx,
            current_idx=nearest_idx,
            centerline_size=len(self.centerline),
        )

        # Stronger incentive to move forward along the centerline.
        # This helps avoid the overly conservative "go slow forever" behavior.
        progress_reward = 1.5 * progress_delta

        # Keep the original tracking penalties.
        lateral_penalty = 0.5 * abs(cross_track_error)
        heading_penalty = 0.2 * abs(heading_error)

        # Slightly reduce the curve-speed penalty.
        # Previous value was 0.05. That made PPO very conservative in curves.
        curve_speed_penalty = (
            0.025 * upcoming_curvature_abs * target_speed * target_speed
        )

        # Small bonus for moving fast only when tracking is healthy.
        # This prevents rewarding reckless speed when the car is far from the path.
        tracking_is_good = (
            abs(cross_track_error) < 0.10
            and abs(heading_error) < 0.15
        )

        speed_tracking_bonus = 0.0
        if tracking_is_good:
            speed_tracking_bonus = 0.05 * float(rl_obs[0])

        crash_penalty = 100.0 if self.crashed else 0.0
        lap_reward = self.lap_bonus if self.lap_completed else 0.0

        reward = (
        progress_reward
        + lap_reward
        + speed_tracking_bonus
        - lateral_penalty
        - heading_penalty
        - curve_speed_penalty
        - crash_penalty
    )

        return float(reward)