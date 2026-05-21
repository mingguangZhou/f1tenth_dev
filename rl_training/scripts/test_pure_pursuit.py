#!/usr/bin/env python3

import argparse

from rl_training.centerline_utils import load_centerline_csv
from rl_training.pure_pursuit import compute_pure_pursuit_steering


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--centerline_csv",
        required=True,
        help="Path to centerline_points_smooth.csv",
    )
    parser.add_argument(
        "--lookahead_distance",
        type=float,
        default=1.0,
        help="Pure pursuit lookahead distance in meters.",
    )
    args = parser.parse_args()

    centerline = load_centerline_csv(args.centerline_csv)

    # Test pose: exactly on the first centerline point and aligned with its yaw.
    # Expected result: steering should usually be small, unless the track curves
    # immediately after the first point.
    p0 = centerline[0]

    steering, lookahead_idx = compute_pure_pursuit_steering(
        car_x=p0.x,
        car_y=p0.y,
        car_yaw=p0.yaw,
        centerline=centerline,
        lookahead_distance=args.lookahead_distance,
    )

    print("=== Pure Pursuit Test ===")
    print(f"Loaded centerline points: {len(centerline)}")
    print(f"Car pose index:           0")
    print(f"Lookahead index:          {lookahead_idx}")
    print(f"Lookahead distance:       {args.lookahead_distance:.3f} m")
    print(f"Steering angle:           {steering:.6f} rad")


if __name__ == "__main__":
    main()
