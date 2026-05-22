#!/usr/bin/env python3

import argparse
import numpy as np

from rl_training.f110_speed_env import F110SpeedEnv
from rl_training.speed_policies import curvature_based_speed


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--map_path", required=True)
    parser.add_argument("--map_ext", required=True)
    parser.add_argument("--centerline_csv", required=True)

    parser.add_argument("--sx", type=float, required=True)
    parser.add_argument("--sy", type=float, required=True)
    parser.add_argument("--stheta", type=float, required=True)

    parser.add_argument("--steps", type=int, default=3000)
    parser.add_argument("--min_speed", type=float, default=0.5)
    parser.add_argument("--max_speed", type=float, default=4.0)
    parser.add_argument("--curvature_gain", type=float, default=2.0)

    args = parser.parse_args()

    env = F110SpeedEnv(
        map_path=args.map_path,
        map_ext=args.map_ext,
        centerline_csv=args.centerline_csv,
        start_pose=(args.sx, args.sy, args.stheta),
        min_speed=args.min_speed,
        max_speed=args.max_speed,
        max_episode_steps=args.steps,
    )

    obs = env.reset()

    total_reward = 0.0
    speed_sum = 0.0
    executed_steps = 0

    print("=== Rule-Based Speed Baseline Evaluation ===")
    print(f"Initial observation: {obs}")

    for step in range(args.steps):
        upcoming_curvature_abs = float(obs[3])

        speed_cmd = curvature_based_speed(
            upcoming_curvature_abs=upcoming_curvature_abs,
            min_speed=args.min_speed,
            max_speed=args.max_speed,
            curvature_gain=args.curvature_gain,
        )

        action = np.array([speed_cmd], dtype=np.float32)
        obs, reward, done, info = env.step(action)

        total_reward += reward
        speed_sum += float(obs[0])
        executed_steps += 1

        if step % 50 == 0:
            print(
                f"step={step:04d} "
                f"cmd_speed={speed_cmd: .3f} "
                f"actual_speed={obs[0]: .3f} "
                f"cte={obs[1]: .3f} "
                f"heading_err={obs[2]: .3f} "
                f"curv={obs[3]: .3f} "
                f"reward={reward: .3f}"
            )

        if done:
            break

    avg_speed = speed_sum / max(executed_steps, 1)

    print("")
    print("=== Evaluation Summary ===")
    print(f"Executed steps:  {executed_steps}")
    print(f"Total reward:    {total_reward:.3f}")
    print(f"Average speed:   {avg_speed:.3f} m/s")
    print(f"Lap completed:   {env.lap_completed}")
    print(f"Crashed:         {env.crashed}")
    print(f"Timeout:         {env.timeout}")


if __name__ == "__main__":
    main()
