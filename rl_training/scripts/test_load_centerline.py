#!/usr/bin/env python3

import argparse

from rl_training.centerline_utils import (
    load_centerline_csv,
    find_nearest_centerline_index,
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

    print("Loaded centerline points:", len(centerline))
    print("Columns: index, x, y, yaw, curvature, curvature_abs")
    print("First point:", centerline[0])
    print("Last point:", centerline[-1])

    test_x = centerline[0].x
    test_y = centerline[0].y

    nearest_idx = find_nearest_centerline_index(
        test_x,
        test_y,
        centerline,
    )

    print("Nearest index to first point:", nearest_idx)


if __name__ == "__main__":
    main()
