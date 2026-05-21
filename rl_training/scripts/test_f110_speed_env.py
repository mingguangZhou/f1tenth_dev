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
    parser.add_argument("--speed", type=float, default=1.5)

    args = parser.parse_args()

    env = F110SpeedEnv(
        map_path=args.map_path,
        map_ext=args.map_ext,
        centerline_csv=args.centerline_csv,
        start_pose=(args.sx, args.sy, args.stheta),
    )

    obs = env.reset()

    print("=== F110 Speed Env Test ===")
    print("Initial RL observation:")
    print(obs)

    total_reward = 0.0

    for i in range(args.steps):
        action = np.array([args.speed], dtype=np.float32)

        obs, reward, done, info = env.step(action)
        total_reward += reward

        if i % 20 == 0:
            print(
                f"step={i:04d} "
                f"speed={obs[0]: .3f} "
                f"cte={obs[1]: .3f} "
                f"heading_err={obs[2]: .3f} "
                f"curv={obs[3]: .3f} "
                f"reward={reward: .3f}"
            )

        if done:
            print(f"Episode ended at step {i}.")
            break

    print("")
    print(f"Total reward: {total_reward:.3f}")


if __name__ == "__main__":
    main()
