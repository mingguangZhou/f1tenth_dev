#!/usr/bin/env python3

import argparse
import numpy as np

from rl_training.f110_speed_env import F110SpeedEnv


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--map_path", required=True)
    parser.add_argument("--map_ext", required=True)
    parser.add_argument("--centerline_csv", required=True)

    parser.add_argument("--sx", type=float, required=True)
    parser.add_argument("--sy", type=float, required=True)
    parser.add_argument("--stheta", type=float, required=True)

    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--correction_action", type=float, default=0.0)
    parser.add_argument("--min_speed", type=float, default=0.5)
    parser.add_argument("--max_speed", type=float, default=4.0)
    parser.add_argument("--max_speed_index_delta", type=float, default=0.05)
    parser.add_argument("--curvature_gain", type=float, default=2.0)
    parser.add_argument("--residual_correction_scale", type=float, default=0.25)
    parser.add_argument("--rule_curvature_lookahead_points", type=int, default=3)
    parser.add_argument("--model_curvature_short_points", type=int, default=10)
    parser.add_argument("--model_curvature_mid_points", type=int, default=40)
    parser.add_argument("--model_curvature_long_points", type=int, default=80)

    args = parser.parse_args()

    env = F110SpeedEnv(
        map_path=args.map_path,
        map_ext=args.map_ext,
        centerline_csv=args.centerline_csv,
        start_pose=(args.sx, args.sy, args.stheta),
        min_speed=args.min_speed,
        max_speed=args.max_speed,
        max_speed_index_delta=args.max_speed_index_delta,
        curvature_gain=args.curvature_gain,
        residual_correction_scale=args.residual_correction_scale,
        rule_curvature_lookahead_points=args.rule_curvature_lookahead_points,
        model_curvature_short_points=args.model_curvature_short_points,
        model_curvature_mid_points=args.model_curvature_mid_points,
        model_curvature_long_points=args.model_curvature_long_points,
    )

    obs = env.reset()

    print("=== F110 Speed Env Test ===")
    print("Initial RL observation:")
    print(obs)

    total_reward = 0.0

    for i in range(args.steps):
        # Action is residual correction_action in [-1, 1].
        # 0.0 follows the rule-based speed reference.
        action = np.array([args.correction_action], dtype=np.float32)

        obs, reward, done, info = env.step(action)
        total_reward += reward

        if i % 20 == 0:
            print(
                f"step={i:04d} "
                f"corr_action={args.correction_action: .3f} "
                f"rule_idx={info.get('rule_speed_index', 0.0): .3f} "
                f"speed_index_req={info.get('requested_speed_index', 0.0): .3f} "
                f"speed_index_exec={info.get('executed_speed_index', 0.0): .3f} "
                f"target_speed={info.get('target_speed_mps', 0.0): .3f} "
                f"actual_speed_index={obs[0]: .3f} "
                f"cte={obs[1]: .3f} "
                f"heading_err={obs[2]: .3f} "
                f"curv_s={obs[3]: .3f} curv_m={obs[4]: .3f} curv_l={obs[5]: .3f} "
                f"reward={reward: .3f}"
            )

        if done:
            print(f"Episode ended at step {i}.")
            break

    print("")
    print(f"Total reward: {total_reward:.3f}")


if __name__ == "__main__":
    main()
