#!/usr/bin/env python3

import argparse
import numpy as np

from stable_baselines3 import PPO

from rl_training.f110_racing_env import F110RacingEnv


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--model_path", required=True)

    parser.add_argument("--map_path", required=True)
    parser.add_argument("--map_ext", required=True)
    parser.add_argument("--centerline_csv", required=True)

    parser.add_argument("--sx", type=float, required=True)
    parser.add_argument("--sy", type=float, required=True)
    parser.add_argument("--stheta", type=float, required=True)

    parser.add_argument("--steps", type=int, default=3000)
    parser.add_argument("--min_speed", type=float, default=0.5)
    parser.add_argument("--max_speed", type=float, default=4.0)
    parser.add_argument("--max_lateral_offset", type=float, default=0.35)
    parser.add_argument("--max_speed_delta_per_step", type=float, default=0.15)
    parser.add_argument("--max_lateral_delta_per_step", type=float, default=0.03)

    args = parser.parse_args()

    env = F110RacingEnv(
        map_path=args.map_path,
        map_ext=args.map_ext,
        centerline_csv=args.centerline_csv,
        start_pose=(args.sx, args.sy, args.stheta),
        min_speed=args.min_speed,
        max_speed=args.max_speed,
        max_lateral_offset=args.max_lateral_offset,
        max_speed_delta_per_step=args.max_speed_delta_per_step,
        max_lateral_delta_per_step=args.max_lateral_delta_per_step,
        max_episode_steps=args.steps,
        use_speed_dependent_lookahead=True,
    )

    model = PPO.load(args.model_path)
    obs = env.reset()

    total_reward = 0.0
    speed_sum = 0.0
    executed_steps = 0

    print("=== PPO Racing Policy Evaluation ===")
    print(f"Initial observation: {obs}")

    for step in range(args.steps):
        action, _ = model.predict(obs, deterministic=True)
        action = np.array(
            [
                np.clip(float(action[0]), 0.0, 1.0),
                np.clip(float(action[1]), -1.0, 1.0),
            ],
            dtype=np.float32,
        )

        obs, reward, done, info = env.step(action)

        total_reward += reward
        speed_sum += float(obs[0])
        executed_steps += 1

        if step % 50 == 0:
            print(
                f"step={step:04d} "
                f"a_speed={action[0]: .3f} "
                f"a_lat={action[1]: .3f} "
                f"target_speed={env.last_target_speed: .3f} "
                f"lat_offset={env.last_lateral_offset: .3f} "
                f"steer={env.last_steering: .3f} "
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
    print("=== PPO Racing Evaluation Summary ===")
    print(f"Executed steps:     {executed_steps}")
    print(f"Total reward:       {total_reward:.3f}")
    print(f"Average speed:      {avg_speed:.3f} m/s")
    print(f"Final target speed: {env.last_target_speed:.3f} m/s")
    print(f"Final lat offset:   {env.last_lateral_offset:.3f} m")
    print(f"Lap completed:      {env.lap_completed}")
    print(f"Crashed:            {env.crashed}")
    print(f"Timeout:            {env.timeout}")


if __name__ == "__main__":
    main()
