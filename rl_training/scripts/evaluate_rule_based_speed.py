#!/usr/bin/env python3

import argparse
import numpy as np
from rl_training.f110_speed_env import F110SpeedEnv


def add_common_env_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--steps", type=int, default=3000)
    parser.add_argument("--min_speed", type=float, default=0.5)
    parser.add_argument("--max_speed", type=float, default=4.0)
    parser.add_argument("--max_speed_index_delta", type=float, default=0.05, help="Deprecated compatibility arg")
    parser.add_argument("--max_speed_delta_per_step_mps", type=float, default=0.10)
    parser.add_argument("--max_delta_speed_mps", type=float, default=0.30)
    parser.add_argument("--residual_correction_scale", type=float, default=0.25, help="Deprecated compatibility arg")
    parser.add_argument("--curvature_gain", type=float, default=2.0)
    parser.add_argument("--rule_curvature_lookahead_points", type=int, default=3)
    parser.add_argument("--model_curvature_short_points", type=int, default=10)
    parser.add_argument("--model_curvature_mid_points", type=int, default=40)
    parser.add_argument("--model_curvature_long_points", type=int, default=80)
    parser.add_argument("--target_lap_steps", type=int, default=5000)
    parser.add_argument("--random_start_along_centerline", action="store_true")
    parser.add_argument("--random_start_min_index", type=int, default=-1)
    parser.add_argument("--random_start_max_index", type=int, default=-1)
    parser.add_argument("--start_lateral_noise_std", type=float, default=0.0)
    parser.add_argument("--start_lateral_noise_max", type=float, default=0.05)
    parser.add_argument("--start_yaw_noise_std", type=float, default=0.0)
    parser.add_argument("--start_yaw_noise_max", type=float, default=0.05)
    parser.add_argument("--start_xy_noise_std", type=float, default=0.0)
    parser.add_argument("--start_xy_noise_max", type=float, default=0.05)
    parser.add_argument("--obs_cte_noise_std", type=float, default=0.0)
    parser.add_argument("--obs_heading_noise_std", type=float, default=0.0)
    parser.add_argument("--obs_speed_noise_std", type=float, default=0.0)

    # High-speed residual training: separate the conservative rule-based
    # reference range from the physical command clamp.
    parser.add_argument("--rule_min_speed_mps", type=float, default=None)
    parser.add_argument("--rule_max_speed_mps", type=float, default=None)

    # Optional early failure when the car has effectively lost the raceline.
    parser.add_argument("--enable_bad_tracking_termination", action="store_true")
    parser.add_argument("--bad_tracking_min_steps", type=int, default=50)
    parser.add_argument("--bad_tracking_cte_threshold", type=float, default=0.75)
    parser.add_argument("--bad_tracking_heading_threshold", type=float, default=0.90)

    # Reward shaping constants exposed for high-speed experiments.
    parser.add_argument("--crash_penalty_value", type=float, default=1000.0)
    parser.add_argument("--timeout_penalty_value", type=float, default=500.0)
    parser.add_argument("--target_speed_smoothness_weight", type=float, default=0.04)
    parser.add_argument("--random_seed", type=int, default=None)


def make_env(args) -> F110SpeedEnv:
    return F110SpeedEnv(
        map_path=args.map_path,
        map_ext=args.map_ext,
        centerline_csv=args.centerline_csv,
        start_pose=(args.sx, args.sy, args.stheta),
        min_speed=args.min_speed,
        max_speed=args.max_speed,
        max_speed_index_delta=args.max_speed_index_delta,
        max_speed_delta_per_step_mps=args.max_speed_delta_per_step_mps,
        max_delta_speed_mps=args.max_delta_speed_mps,
        residual_correction_scale=args.residual_correction_scale,
        max_episode_steps=args.steps,
        target_lap_steps=args.target_lap_steps,
        curvature_gain=args.curvature_gain,
        rule_curvature_lookahead_points=args.rule_curvature_lookahead_points,
        model_curvature_short_points=args.model_curvature_short_points,
        model_curvature_mid_points=args.model_curvature_mid_points,
        model_curvature_long_points=args.model_curvature_long_points,
        random_start_along_centerline=args.random_start_along_centerline,
        random_start_min_index=args.random_start_min_index,
        random_start_max_index=args.random_start_max_index,
        start_lateral_noise_std=args.start_lateral_noise_std,
        start_lateral_noise_max=args.start_lateral_noise_max,
        start_yaw_noise_std=args.start_yaw_noise_std,
        start_yaw_noise_max=args.start_yaw_noise_max,
        start_xy_noise_std=args.start_xy_noise_std,
        start_xy_noise_max=args.start_xy_noise_max,
        obs_cte_noise_std=args.obs_cte_noise_std,
        obs_heading_noise_std=args.obs_heading_noise_std,
        obs_speed_noise_std=args.obs_speed_noise_std,

        rule_min_speed_mps=args.rule_min_speed_mps,
        rule_max_speed_mps=args.rule_max_speed_mps,
        enable_bad_tracking_termination=args.enable_bad_tracking_termination,
        bad_tracking_min_steps=args.bad_tracking_min_steps,
        bad_tracking_cte_threshold=args.bad_tracking_cte_threshold,
        bad_tracking_heading_threshold=args.bad_tracking_heading_threshold,
        crash_penalty_value=args.crash_penalty_value,
        timeout_penalty_value=args.timeout_penalty_value,
        target_speed_smoothness_weight=args.target_speed_smoothness_weight,
        random_seed=args.random_seed,
        use_speed_dependent_lookahead=True,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map_path", required=True)
    parser.add_argument("--map_ext", required=True)
    parser.add_argument("--centerline_csv", required=True)
    parser.add_argument("--sx", type=float, required=True)
    parser.add_argument("--sy", type=float, required=True)
    parser.add_argument("--stheta", type=float, required=True)
    add_common_env_args(parser)
    args = parser.parse_args()

    env = make_env(args)
    obs = env.reset()

    total_reward = 0.0
    speed_sum = 0.0
    executed_steps = 0

    print("=== Rule-Based Speed Baseline Evaluation ===")
    print(f"Initial observation: {obs}")

    for step in range(args.steps):
        action = np.array([0.0], dtype=np.float32)
        obs, reward, done, info = env.step(action)
        total_reward += reward
        speed_sum += float(info.get("target_speed_mps", 0.0))
        executed_steps += 1

        if step % 50 == 0:
            print(
                f"step={step:04d} "
                f"action= 0.000 "
                f"delta_v={info.get('delta_speed_mps', 0.0): .3f} "
                f"rule_v={info.get('rule_speed_mps', 0.0): .3f} "
                f"target_v={info.get('target_speed_mps', 0.0): .3f} "
                f"actual_v={obs[0]: .3f} "
                f"cte={obs[2]: .3f} "
                f"heading_err={obs[3]: .3f} "
                f"curv_s={obs[4]: .3f} curv_m={obs[5]: .3f} curv_l={obs[6]: .3f} "
                f"reward={reward: .3f}"
            )
        if done:
            break

    avg_speed = speed_sum / max(executed_steps, 1)
    print("")
    print("=== Evaluation Summary ===")
    print(f"Executed steps:  {executed_steps}")
    print(f"Total reward:    {total_reward:.3f}")
    print(f"Average target speed: {avg_speed:.3f} m/s")
    print(f"Lap completed:   {env.lap_completed}")
    print(f"Crashed:         {env.crashed}")
    print(f"Timeout:         {env.timeout}")
    print(f"Bad tracking:    {getattr(env, 'bad_tracking_failure', False)}")


if __name__ == "__main__":
    main()
