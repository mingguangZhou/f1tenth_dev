#!/usr/bin/env python3

import argparse
import os

from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor

from rl_training.f110_speed_env import F110SpeedEnv


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--map_path", required=True)
    parser.add_argument("--map_ext", required=True)
    parser.add_argument("--centerline_csv", required=True)

    parser.add_argument("--sx", type=float, required=True)
    parser.add_argument("--sy", type=float, required=True)
    parser.add_argument("--stheta", type=float, required=True)

    parser.add_argument("--total_timesteps", type=int, default=50000)
    parser.add_argument("--model_dir", default="models")
    parser.add_argument("--model_name", default="ppo_speed_agent")

    parser.add_argument("--min_speed", type=float, default=0.5)
    parser.add_argument("--max_speed", type=float, default=4.0)

    args = parser.parse_args()

    os.makedirs(args.model_dir, exist_ok=True)

    env = F110SpeedEnv(
        map_path=args.map_path,
        map_ext=args.map_ext,
        centerline_csv=args.centerline_csv,
        start_pose=(args.sx, args.sy, args.stheta),
        min_speed=args.min_speed,
        max_speed=args.max_speed,
        max_episode_steps=3000,
        use_speed_dependent_lookahead=True,
    )

    env = Monitor(env)

    model = PPO(
        policy="MlpPolicy",
        env=env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=1024,
        batch_size=64,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        tensorboard_log=os.path.join(args.model_dir, "tensorboard"),
    )

    model.learn(total_timesteps=args.total_timesteps)

    save_path = os.path.join(args.model_dir, args.model_name)
    model.save(save_path)

    print("")
    print("Training finished.")
    print(f"Saved model to: {save_path}.zip")


if __name__ == "__main__":
    main()
