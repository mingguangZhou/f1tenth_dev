#!/usr/bin/env python3
"""Run Phase 1-9 centerline generation, then a fresh manual raceline editor."""

import subprocess
import sys
from pathlib import Path


def run(cmd):
    print("\n>>> " + " ".join(str(x) for x in cmd))
    subprocess.run(cmd, check=True)


def main():
    if len(sys.argv) != 3:
        print("Usage: python3 run_manual_centerline_and_raceline.py <map_image> <map.yaml>")
        return

    here = Path(__file__).resolve().parent
    map_path = sys.argv[1]
    yaml_path = sys.argv[2]

    # A full run is normally used for a new/regenerated map, so explicitly
    # start the manual editor fresh. Running manual_raceline_generator.py by
    # itself resumes the most recent compatible manual session by default.
    run([sys.executable, str(here / "centerline_reference_generator.py"), map_path, yaml_path])
    run([sys.executable, str(here / "manual_raceline_generator.py"), map_path, yaml_path, "--fresh"])


if __name__ == "__main__":
    main()
