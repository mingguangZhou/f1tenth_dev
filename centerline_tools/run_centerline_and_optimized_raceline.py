#!/usr/bin/env python3
"""Compatibility command for the map-isolated raceline workbench.

The two map arguments remain supported; optional workbench flags are forwarded.
"""

from raceline_workbench import main


if __name__ == "__main__":
    raise SystemExit(main())
