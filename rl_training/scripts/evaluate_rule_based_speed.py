#!/usr/bin/env python3

import argparse
import numpy as np
from rl_training.f110_speed_env import F110SpeedEnv
from rl_training.config_utils import parse_args_with_config, apply_start_centerline_idx

REWARD_TERM_KEYS = [
    "reward_progress",
    "reward_lap",
    "reward_early_finish",
    "penalty_tracking",
    "penalty_curvature_speed_section",
    "penalty_target_speed_smoothness",
    "penalty_residual_smoothness",
    "penalty_residual_excess",
    "penalty_time",
    "penalty_crash",
    "penalty_timeout",
]

def make_reward_accumulator():
    return {key: 0.0 for key in REWARD_TERM_KEYS}

def accumulate_reward_terms(acc, info):
    for key in REWARD_TERM_KEYS:
        acc[key] += float(info.get(key, 0.0))

def print_reward_breakdown(acc, total_reward):
    print("")
    print("=== Reward Breakdown ===")
    print(f"reward_progress:                  +{acc['reward_progress']:.3f}")
    print(f"reward_lap:                       +{acc['reward_lap']:.3f}")
    print(f"reward_early_finish:              +{acc['reward_early_finish']:.3f}")
    print(f"penalty_tracking:                 -{acc['penalty_tracking']:.3f}")
    print(f"penalty_curvature_speed_section:  -{acc['penalty_curvature_speed_section']:.3f}")
    print(f"penalty_target_speed_smoothness:  -{acc['penalty_target_speed_smoothness']:.3f}")
    print(f"penalty_residual_smoothness:      -{acc['penalty_residual_smoothness']:.3f}")
    print(f"penalty_residual_excess:          -{acc['penalty_residual_excess']:.3f}")
    print(f"penalty_time:                     -{acc['penalty_time']:.3f}")
    print(f"penalty_crash:                    -{acc['penalty_crash']:.3f}")
    print(f"penalty_timeout:                  -{acc['penalty_timeout']:.3f}")
    print(f"reward_total_check:                {total_reward:.3f}")


def add_common_env_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--steps", type=int, default=3000)
    parser.add_argument("--min_speed", type=float, default=0.5, help="Legacy alias for command_speed_min_mps")
    parser.add_argument("--max_speed", type=float, default=4.0, help="Legacy alias for command_speed_max_mps")
    parser.add_argument("--command_speed_min_mps", type=float, default=None)
    parser.add_argument("--command_speed_max_mps", type=float, default=None)
    parser.add_argument("--max_speed_index_delta", type=float, default=0.05, help="Deprecated compatibility arg")
    parser.add_argument("--max_speed_delta_per_step_mps", type=float, default=0.10)
    parser.add_argument("--max_delta_speed_mps", type=float, default=0.30)
    parser.add_argument("--residual_correction_scale", type=float, default=0.25, help="Deprecated compatibility arg")
    parser.add_argument("--curvature_gain", type=float, default=2.0)
    parser.add_argument("--rule_curvature_lookahead_points", type=int, default=3)
    parser.add_argument("--model_curvature_short_points", type=int, default=10)
    parser.add_argument("--model_curvature_mid_points", type=int, default=40)
    parser.add_argument("--model_curvature_long_points", type=int, default=80)
    parser.add_argument("--rule_speed_curvature_preview_m", type=float, default=None)
    parser.add_argument("--model_curvature_short_preview_m", type=float, default=None)
    parser.add_argument("--model_curvature_mid_preview_m", type=float, default=None)
    parser.add_argument("--model_curvature_long_preview_m", type=float, default=None)
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
    parser.add_argument("--rule_min_speed_mps", type=float, default=None, help="Legacy alias for rule_curve_min_speed_mps")
    parser.add_argument("--rule_max_speed_mps", type=float, default=None, help="Legacy alias for rule_straight_speed_mps")
    parser.add_argument("--rule_curve_min_speed_mps", type=float, default=None)
    parser.add_argument("--rule_straight_speed_mps", type=float, default=None)
    parser.add_argument("--rule_speed_curvature_gain", type=float, default=None)

    # Optional early failure when the car has effectively lost the raceline.
    parser.add_argument("--enable_bad_tracking_termination", action="store_true")
    parser.add_argument("--bad_tracking_min_steps", type=int, default=50)
    parser.add_argument("--bad_tracking_cte_threshold", type=float, default=0.75)
    parser.add_argument("--bad_tracking_heading_threshold", type=float, default=0.90)

    # Reward shaping constants exposed for high-speed experiments.
    parser.add_argument("--crash_penalty_value", type=float, default=1000.0)
    parser.add_argument("--timeout_penalty_value", type=float, default=500.0)
    parser.add_argument("--target_speed_smoothness_weight", type=float, default=0.04)
    parser.add_argument("--reward_curvature_section_start_points", type=int, default=2)
    parser.add_argument("--reward_curvature_section_end_points", type=int, default=40)
    parser.add_argument("--reward_curvature_section_start_m", type=float, default=None)
    parser.add_argument("--reward_curvature_section_end_m", type=float, default=None)
    parser.add_argument("--curvature_speed_section_weight", type=float, default=0.006)
    parser.add_argument("--residual_smoothness_weight", type=float, default=0.08)
    parser.add_argument("--residual_free_band_mps", type=float, default=0.8)
    parser.add_argument("--residual_excess_weight", type=float, default=0.05)
    # Optional runtime/inference safety gate for the learned residual.
    parser.add_argument("--enable_rl_gate", action="store_true")
    parser.add_argument("--rl_gate_enable_cte", type=float, default=0.25)
    parser.add_argument("--rl_gate_enable_heading", type=float, default=0.20)
    parser.add_argument("--rl_gate_disable_cte", type=float, default=0.45)
    parser.add_argument("--rl_gate_disable_heading", type=float, default=0.35)
    parser.add_argument("--rl_gate_enable_count", type=int, default=10)
    parser.add_argument("--rl_gate_disable_count", type=int, default=3)
    parser.add_argument("--rl_gate_fade_in_step", type=float, default=0.05)
    parser.add_argument("--rl_gate_fade_out_step", type=float, default=0.10)

    parser.add_argument("--wheelbase_m", type=float, default=None)
    parser.add_argument("--steering_max_deg", type=float, default=None)
    parser.add_argument("--fixed_steering_lookahead_m", type=float, default=None)
    parser.add_argument("--use_speed_dependent_steering_lookahead", action="store_true", default=None)
    parser.add_argument("--steering_min_lookahead_m", type=float, default=None)
    parser.add_argument("--steering_max_lookahead_m", type=float, default=None)
    parser.add_argument("--steering_lookahead_speed_gain", type=float, default=None)
    parser.add_argument("--min_forward_point_x_m", type=float, default=0.05)

    parser.add_argument("--random_seed", type=int, default=None)


def make_env(args) -> F110SpeedEnv:
    return F110SpeedEnv(
        map_path=args.map_path,
        map_ext=args.map_ext,
        centerline_csv=args.centerline_csv,
        start_pose=(args.sx, args.sy, args.stheta),
        command_speed_min_mps=args.command_speed_min_mps,
        command_speed_max_mps=args.command_speed_max_mps,
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
        rule_speed_curvature_preview_m=args.rule_speed_curvature_preview_m,
        model_curvature_short_preview_m=args.model_curvature_short_preview_m,
        model_curvature_mid_preview_m=args.model_curvature_mid_preview_m,
        model_curvature_long_preview_m=args.model_curvature_long_preview_m,
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
        rule_curve_min_speed_mps=args.rule_curve_min_speed_mps,
        rule_straight_speed_mps=args.rule_straight_speed_mps,
        rule_speed_curvature_gain=args.rule_speed_curvature_gain,
        enable_bad_tracking_termination=args.enable_bad_tracking_termination,
        bad_tracking_min_steps=args.bad_tracking_min_steps,
        bad_tracking_cte_threshold=args.bad_tracking_cte_threshold,
        bad_tracking_heading_threshold=args.bad_tracking_heading_threshold,
        crash_penalty_value=args.crash_penalty_value,
        timeout_penalty_value=args.timeout_penalty_value,
        target_speed_smoothness_weight=args.target_speed_smoothness_weight,
        reward_curvature_section_start_points=args.reward_curvature_section_start_points,
        reward_curvature_section_end_points=args.reward_curvature_section_end_points,
        reward_curvature_section_start_m=args.reward_curvature_section_start_m,
        reward_curvature_section_end_m=args.reward_curvature_section_end_m,
        curvature_speed_section_weight=args.curvature_speed_section_weight,
        residual_smoothness_weight=args.residual_smoothness_weight,
        residual_free_band_mps=args.residual_free_band_mps,
        residual_excess_weight=args.residual_excess_weight,
        enable_rl_gate=args.enable_rl_gate,
        rl_gate_enable_cte=args.rl_gate_enable_cte,
        rl_gate_enable_heading=args.rl_gate_enable_heading,
        rl_gate_disable_cte=args.rl_gate_disable_cte,
        rl_gate_disable_heading=args.rl_gate_disable_heading,
        rl_gate_enable_count=args.rl_gate_enable_count,
        rl_gate_disable_count=args.rl_gate_disable_count,
        rl_gate_fade_in_step=args.rl_gate_fade_in_step,
        rl_gate_fade_out_step=args.rl_gate_fade_out_step,
        random_seed=args.random_seed,
        wheelbase_m=args.wheelbase_m,
        steering_max_deg=args.steering_max_deg,
        fixed_steering_lookahead_m=args.fixed_steering_lookahead_m,
        use_speed_dependent_steering_lookahead=args.use_speed_dependent_steering_lookahead,
        steering_min_lookahead_m=args.steering_min_lookahead_m,
        steering_max_lookahead_m=args.steering_max_lookahead_m,
        steering_lookahead_speed_gain=args.steering_lookahead_speed_gain,
        min_forward_point_x_m=args.min_forward_point_x_m,
        use_speed_dependent_lookahead=True,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map_path", default=None)
    parser.add_argument("--map_ext", default=None)
    parser.add_argument("--centerline_csv", default=None)
    parser.add_argument("--sx", type=float, default=None)
    parser.add_argument("--sy", type=float, default=None)
    parser.add_argument("--stheta", type=float, default=None)
    parser.add_argument("--start_centerline_idx", type=int, default=-1, help="Optional raceline CSV index used to set sx/sy/stheta for same-start tests")
    add_common_env_args(parser)
    args = parse_args_with_config(
        parser,
        required_keys=["map_path", "map_ext", "centerline_csv", "sx", "sy", "stheta"],
    )
    args = apply_start_centerline_idx(args)

    env = make_env(args)
    obs = env.reset()

    total_reward = 0.0
    reward_acc = make_reward_accumulator()
    speed_sum = 0.0
    executed_steps = 0

    print("=== Rule-Based Speed Baseline Evaluation ===")
    print(f"Initial observation: {obs}")

    for step in range(args.steps):
        action = np.array([0.0], dtype=np.float32)
        obs, reward, done, info = env.step(action)
        total_reward += reward
        accumulate_reward_terms(reward_acc, info)
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
    print_reward_breakdown(reward_acc, total_reward)


if __name__ == "__main__":
    main()
