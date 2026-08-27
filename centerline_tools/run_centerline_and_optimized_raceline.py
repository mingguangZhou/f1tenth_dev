#!/usr/bin/env python3
"""Generate a centerline, then optimize a globally smooth raceline."""

import subprocess
import sys
from pathlib import Path


def run(command, working_directory):
    """Run one offline pipeline stage and stop on failure."""
    print("\n>>> " + " ".join(str(item) for item in command))
    subprocess.run(command, check=True, cwd=working_directory)


def main():
    """Run the Phase-9 generator followed by the headless optimizer."""
    if len(sys.argv) != 3:
        print(
            "Usage: python3 run_centerline_and_optimized_raceline.py "
            "<map_image> <map.yaml>"
        )
        return 2

    root = Path(__file__).resolve().parent
    output = root / "centerline_output"
    map_image = Path(sys.argv[1]).resolve()
    map_yaml = Path(sys.argv[2]).resolve()
    run(
        [
            sys.executable,
            str(root / "centerline_reference_generator.py"),
            str(map_image),
            str(map_yaml),
        ],
        root,
    )
    run(
        [
            sys.executable,
            str(root / "optimize_global_raceline.py"),
            "--centerline",
            str(output / "centerline_points_smooth.csv"),
            "--drivable-mask",
            str(output / "drivable_region.npy"),
            "--map-yaml",
            str(map_yaml),
            "--config",
            str(root / "config" / "global_raceline_optimizer.yaml"),
            "--output",
            str(output / "raceline_points_optimized.csv"),
            "--report",
            str(output / "raceline_points_optimized_validation.yaml"),
            "--debug-plot",
            str(output / "debug_global_raceline_optimization.png"),
        ],
        root,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
