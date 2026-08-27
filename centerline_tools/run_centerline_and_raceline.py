#!/usr/bin/env python3
"""Run the separated centerline and raceline pipeline in order."""

import subprocess
import sys
from pathlib import Path


def run(cmd):
    print("\n>>> " + " ".join(cmd))
    subprocess.run(cmd, check=True)


def main():
    if len(sys.argv) != 3:
        print("Usage: python3 run_centerline_and_raceline.py <map_image> <map.yaml>")
        return

    here = Path(__file__).resolve().parent
    map_path = sys.argv[1]
    yaml_path = sys.argv[2]

    run([sys.executable, str(here / "centerline_reference_generator.py"), map_path, yaml_path])
    run([sys.executable, str(here / "raceline_generator.py"), map_path, yaml_path])


if __name__ == "__main__":
    main()
