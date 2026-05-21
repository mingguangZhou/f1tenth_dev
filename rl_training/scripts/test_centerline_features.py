#!/usr/bin/env python3

import argparse
import math

from rl_training.centerline_utils import (
    load_centerline_csv,
    get_centerline_state_features,
    compute_centerline_progress_delta,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--centerline_csv",
        required=True,
        help="Path to centerline_points_smooth.csv",
    )
    args = parser.parse_args()

    centerline = load_centerline_csv(args.centerline_csv)

    # Use the first centerline point as a fake vehicle pose.
    # This should produce near-zero lateral and heading error.
    p0 = centerline[0]

    car_x = p0.x
    car_y = p0.y
    car_yaw = p0.yaw
    car_speed = 1.5

    features, nearest_idx = get_centerline_state_features(
        car_x=car_x,
        car_y=car_y,
        car_yaw=car_yaw,
        car_speed=car_speed,
        centerline=centerline,
        curvature_lookahead_points=20,
    )

    print("=== Centerline Feature Test ===")
    print(f"Loaded centerline points: {len(centerline)}")
    print(f"Nearest index: {nearest_idx}")
    print("")
    print("RL observation features:")
    print(f"  car_speed:              {features[0]: .4f} m/s")
    print(f"  cross_track_error:      {features[1]: .4f} m")
    print(f"  heading_error:          {features[2]: .4f} rad")
    print(f"  upcoming_curvature_abs: {features[3]: .4f} 1/m")

    # Test closed-loop progress logic near the wrap-around point.
    previous_idx = len(centerline) - 3
    current_idx = 2

    progress_delta = compute_centerline_progress_delta(
        previous_idx=previous_idx,
        current_idx=current_idx,
        centerline_size=len(centerline),
    )

    print("")
    print("Closed-loop progress test:")
    print(f"  previous_idx:   {previous_idx}")
    print(f"  current_idx:    {current_idx}")
    print(f"  progress_delta: {progress_delta}")

    if abs(features[1]) < 1e-6 and abs(features[2]) < 1e-6:
        print("")
        print("Result: PASS. Geometry features look healthy.")
    else:
        print("")
        print("Result: CHECK. Expected near-zero lateral and heading error.")


if __name__ == "__main__":
    main()
