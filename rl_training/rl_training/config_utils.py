"""Small YAML configuration helpers for RL training/evaluation scripts.

The config format is intentionally simple: nested YAML dictionaries are flattened
into argparse-style option names. For example:

speed:
  max_speed: 10.0

becomes args.max_speed = 10.0.

Command-line arguments always override values loaded from YAML.
"""

from __future__ import annotations

import argparse
from typing import Any, Dict, Iterable

import yaml


def _flatten_dict(data: Dict[str, Any], out: Dict[str, Any]) -> None:
    for key, value in data.items():
        if isinstance(value, dict):
            _flatten_dict(value, out)
        else:
            out[key] = value


def load_yaml_defaults(config_path: str) -> Dict[str, Any]:
    """Load a YAML config and flatten nested groups to argparse defaults."""
    with open(config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    if not isinstance(data, dict):
        raise ValueError(f"Config file must contain a YAML mapping: {config_path}")

    defaults: Dict[str, Any] = {}
    _flatten_dict(data, defaults)
    return defaults


def parse_args_with_config(
    parser: argparse.ArgumentParser,
    required_keys: Iterable[str] = (),
):
    """Parse args with optional --config support.

    The parser should declare normal argparse options first. This helper then:
      1. pre-parses --config,
      2. loads YAML values as parser defaults,
      3. parses the full command line,
      4. checks required_keys after YAML + CLI values are merged.

    CLI values override YAML values because argparse applies explicit command-line
    options after parser defaults.
    """
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--config", default=None, help="YAML config file with default arguments")
    pre_args, _ = pre_parser.parse_known_args()

    parser.add_argument("--config", default=None, help="YAML config file with default arguments")

    if pre_args.config:
        parser.set_defaults(**load_yaml_defaults(pre_args.config))

    args = parser.parse_args()

    missing = []
    for key in required_keys:
        value = getattr(args, key, None)
        if value is None or value == "":
            missing.append(key)
    if missing:
        parser.error(
            "Missing required arguments after applying --config: "
            + ", ".join("--" + key for key in missing)
        )

    return args
