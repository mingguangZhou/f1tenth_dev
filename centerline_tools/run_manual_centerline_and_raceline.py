#!/usr/bin/env python3
"""Run Phase 1-9 centerline generation, then the manual raceline editor."""

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

    run([sys.executable, str(here / "centerline_reference_generator.py"), map_path, yaml_path])
    # A full run has just regenerated Phase 9 centerline data, so always start
    # a fresh manual-raceline session. Standalone manual_raceline_generator.py
    # still resumes the last compatible manual session by default.
    run([sys.executable, str(here / "manual_raceline_generator.py"), map_path, yaml_path, "--fresh"])


if __name__ == "__main__":
    main()