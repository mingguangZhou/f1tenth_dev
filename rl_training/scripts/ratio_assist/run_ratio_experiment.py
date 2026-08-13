#!/usr/bin/env python3
"""Single-entry runner for the ratio-assist speed-RL workflow.

This script does not change the core RL environment or PPO scripts. It reads one
master YAML, generates the small stage-specific YAML expected by the existing
low-level scripts, and then calls those scripts.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

import yaml


STAGES = [
    "rule-highspeed",
    "train-highspeed",
    "continue-highspeed",
    "eval-highspeed",
    "compare-highspeed",
    "sweep-highspeed",
    "rule-reserved",
    "eval-reserved",
    "compare-reserved",
    "sweep-reserved",
]


def load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Master config must be a YAML mapping: {path}")
    return data


def deep_update(dst: Dict[str, Any], src: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    if not src:
        return dst
    for key, value in src.items():
        if isinstance(value, dict) and isinstance(dst.get(key), dict):
            deep_update(dst[key], value)
        else:
            dst[key] = deepcopy(value)
    return dst



def copy_selected(src: Optional[Dict[str, Any]], keys: Iterable[str]) -> Dict[str, Any]:
    src = src or {}
    return {k: deepcopy(src[k]) for k in keys if k in src}

def stage_profile(stage: str) -> str:
    if "reserved" in stage:
        return "reserved_realistic"
    return "highspeed_sim_stress"


def default_model_path(master: Dict[str, Any]) -> str:
    model = master.get("model", {})
    if model.get("model_path"):
        return str(model["model_path"])
    model_dir = str(model.get("model_dir", "models"))
    model_name = str(model.get("model_name", master.get("experiment", {}).get("name", "ppo_speed_agent")))
    if model_name.endswith(".zip"):
        return os.path.join(model_dir, model_name)
    return os.path.join(model_dir, model_name + ".zip")


def make_stage_config(
    master: Dict[str, Any],
    stage: str,
    args: argparse.Namespace,
    assist_gain_override: Optional[float] = None,
    start_idx_override: Optional[int] = None,
    out_dir_override: Optional[str] = None,
) -> Dict[str, Any]:
    profile_name = stage_profile(stage)
    profile = deepcopy(master["profiles"][profile_name])
    shared = master.get("shared", {})
    model = master.get("model", {})
    experiment_name = master.get("experiment", {}).get("name", model.get("model_name", "ppo_speed_agent"))

    is_train = stage in {"train-highspeed", "continue-highspeed"}
    is_compare = stage.startswith("compare") or stage.startswith("sweep")
    is_rule = stage.startswith("rule")

    cfg: Dict[str, Any] = {
        "map": deepcopy(master.get("paths", {})),
        "start": {
            "sx": master.get("start", {}).get("sx", 0.0),
            "sy": master.get("start", {}).get("sy", 0.0),
            "stheta": master.get("start", {}).get("stheta", 0.0),
            "start_centerline_idx": master.get("start", {}).get("start_centerline_idx", 0 if is_compare else -1),
            "random_start_along_centerline": False,
            "random_start_min_index": -1,
            "random_start_max_index": -1,
            "start_lateral_noise_std": 0.0,
            "start_lateral_noise_max": 0.05,
            "start_yaw_noise_std": 0.0,
            "start_yaw_noise_max": 0.05,
            "start_xy_noise_std": 0.0,
            "start_xy_noise_max": 0.05,
        },
        "observation_noise": {
            "obs_cte_noise_std": 0.0,
            "obs_heading_noise_std": 0.0,
            "obs_speed_noise_std": 0.0,
        },
        "speed": deepcopy(profile.get("speed", {})),
        "rl_residual": deepcopy(shared.get("rl_residual", {})),
        "curvature": deepcopy(shared.get("curvature_observation", {})),
        "reward": deepcopy(shared.get("reward", {})),
        "termination": deepcopy(shared.get("termination", {})),
        "gate": deepcopy(shared.get("gate", {})),
        "steering": deepcopy(shared.get("steering", {})),
        "ppo": {
            "model_dir": model.get("model_dir", "models"),
            "model_name": args.model_name or model.get("model_name", experiment_name),
            "load_model_path": args.load_model_path or model.get("load_model_path"),
            "reset_num_timesteps": bool(args.reset_num_timesteps),
        },
        "analysis": {
            "out_dir": out_dir_override or profile.get("analysis", {}).get("out_dir", f"analysis_{profile_name}"),
            "print_every": args.print_every if args.print_every is not None else profile.get("analysis", {}).get("print_every", 100),
        },
        "model": {
            "model_path": args.model_path or default_model_path(master),
        },
        "misc": {
            "random_seed": master.get("misc", {}).get("random_seed"),
        },
    }

    deep_update(cfg["rl_residual"], profile.get("rl", {}))
    if assist_gain_override is not None:
        cfg["rl_residual"]["assist_gain"] = float(assist_gain_override)
    if args.assist_gain is not None:
        cfg["rl_residual"]["assist_gain"] = float(args.assist_gain)

    start_keys = {
        "random_start_along_centerline", "random_start_min_index", "random_start_max_index",
        "start_lateral_noise_std", "start_lateral_noise_max", "start_yaw_noise_std",
        "start_yaw_noise_max", "start_xy_noise_std", "start_xy_noise_max",
    }
    obs_keys = {"obs_cte_noise_std", "obs_heading_noise_std", "obs_speed_noise_std"}
    gate_keys = {"enable_rl_gate"}
    ppo_keys = {
        "total_timesteps", "learning_rate", "n_steps", "batch_size", "gamma",
        "gae_lambda", "clip_range", "ent_coef",
    }

    if is_train:
        train_cfg = profile.get("train", {})
        deep_update(cfg["start"], copy_selected(train_cfg, start_keys))
        deep_update(cfg["observation_noise"], copy_selected(train_cfg, obs_keys))
        deep_update(cfg["gate"], copy_selected(train_cfg, gate_keys))
        deep_update(cfg["ppo"], copy_selected(train_cfg, ppo_keys))
        if args.total_timesteps is not None:
            cfg["ppo"]["total_timesteps"] = int(args.total_timesteps)
        if stage == "train-highspeed":
            cfg["ppo"]["load_model_path"] = None
        if stage == "continue-highspeed" and not cfg["ppo"].get("load_model_path"):
            raise ValueError("continue-highspeed requires --load_model_path or model.load_model_path in the master YAML")
    else:
        eval_cfg = profile.get("eval", {})
        deep_update(cfg["start"], copy_selected(eval_cfg, start_keys))
        deep_update(cfg["observation_noise"], copy_selected(eval_cfg, obs_keys))
        deep_update(cfg["gate"], copy_selected(eval_cfg, gate_keys))
        if "analysis" in stage or is_compare:
            deep_update(cfg["gate"], copy_selected(profile.get("analysis", {}), gate_keys))

    # For rule/eval fixed-start stages, do not force a raceline index unless requested.
    if is_compare:
        cfg["start"]["start_centerline_idx"] = int(
            start_idx_override if start_idx_override is not None else (0 if args.start_centerline_idx is None else args.start_centerline_idx)
        )
    elif args.start_centerline_idx is not None:
        cfg["start"]["start_centerline_idx"] = int(args.start_centerline_idx)

    if args.steps is not None:
        cfg["speed"]["steps"] = int(args.steps)
    if args.model_name:
        cfg["ppo"]["model_name"] = args.model_name
    if args.model_path:
        cfg["model"]["model_path"] = args.model_path

    # Remove training optimizer keys from non-training generated configs; the low-level
    # scripts ignore them, but this keeps generated configs readable.
    if not is_train:
        for k in ["learning_rate", "n_steps", "batch_size", "gamma", "gae_lambda", "clip_range", "ent_coef", "total_timesteps"]:
            cfg["ppo"].pop(k, None)

    return cfg


def write_stage_config(master_path: str, master: Dict[str, Any], stage: str, cfg: Dict[str, Any]) -> str:
    exp_name = str(master.get("experiment", {}).get("name", "ratio_experiment"))
    out_dir = Path("runs") / exp_name / "generated_configs"
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = Path(master_path).stem
    filename = f"{stem}_{stage}.generated.yaml"
    path = out_dir / filename
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    return str(path)


def run_cmd(cmd: List[str], dry_run: bool) -> None:
    print("\n>>> " + " ".join(cmd))
    if not dry_run:
        subprocess.run(cmd, check=True)


def script_for_stage(stage: str) -> str:
    if stage.startswith("rule"):
        return "scripts/evaluate_rule_based_speed.py"
    if stage in {"train-highspeed", "continue-highspeed"}:
        return "scripts/train_ppo_speed.py"
    if stage.startswith("eval"):
        return "scripts/evaluate_ppo_speed.py"
    if stage.startswith("compare") or stage.startswith("sweep"):
        return "scripts/analyze_speed_policy_comparison.py"
    raise ValueError(f"Unsupported stage: {stage}")


def parse_float_list(text: Optional[str], fallback: Iterable[float]) -> List[float]:
    if not text:
        return [float(x) for x in fallback]
    return [float(x.strip()) for x in text.split(",") if x.strip()]


def gain_tag(gain: float) -> str:
    return f"{gain:.2f}".replace(".", "")


def run_one(master_path: str, master: Dict[str, Any], stage: str, args: argparse.Namespace,
            assist_gain: Optional[float] = None, start_idx: Optional[int] = None) -> None:
    out_dir = args.out_dir
    if out_dir is None and (stage.startswith("compare") or stage.startswith("sweep")):
        base_profile = "highspeed" if "highspeed" in stage else "reserved"
        idx = start_idx if start_idx is not None else (0 if args.start_centerline_idx is None else args.start_centerline_idx)
        if assist_gain is not None:
            out_dir = f"analysis_{master['experiment']['name']}_{base_profile}_idx{idx}_gain{gain_tag(assist_gain)}"
        else:
            out_dir = f"analysis_{master['experiment']['name']}_{base_profile}_idx{idx}"

    cfg = make_stage_config(
        master=master,
        stage=stage,
        args=args,
        assist_gain_override=assist_gain,
        start_idx_override=start_idx,
        out_dir_override=out_dir,
    )
    gen_config = write_stage_config(master_path, master, stage, cfg)

    cmd = [sys.executable, script_for_stage(stage), "--config", gen_config]
    if stage.startswith("eval") or stage.startswith("compare") or stage.startswith("sweep"):
        cmd += ["--model_path", str(cfg["model"]["model_path"])]
    if stage.startswith("compare") or stage.startswith("sweep"):
        cmd += ["--out_dir", str(cfg["analysis"]["out_dir"])]
        cmd += ["--start_centerline_idx", str(cfg["start"].get("start_centerline_idx", 0))]
        cmd += ["--print_every", str(cfg["analysis"].get("print_every", 100))]
    if stage in {"train-highspeed", "continue-highspeed"}:
        if cfg["ppo"].get("total_timesteps") is not None:
            cmd += ["--total_timesteps", str(cfg["ppo"]["total_timesteps"])]
        if cfg["ppo"].get("model_name"):
            cmd += ["--model_name", str(cfg["ppo"]["model_name"])]
        if stage == "continue-highspeed" and cfg["ppo"].get("load_model_path"):
            cmd += ["--load_model_path", str(cfg["ppo"]["load_model_path"])]
        if bool(cfg["ppo"].get("reset_num_timesteps", False)):
            cmd += ["--reset_num_timesteps"]
    run_cmd(cmd, args.dry_run)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the ratio-assist speed-RL pipeline from one master YAML.")
    parser.add_argument("--config", default="configs/ratio_assist/V1_reward_ratio_spielberg_master.yaml")
    parser.add_argument("--stage", required=True, choices=STAGES)
    parser.add_argument("--model_path", default=None, help="Override model .zip path for eval/compare stages")
    parser.add_argument("--model_name", default=None, help="Override output model name for train/continue stages")
    parser.add_argument("--load_model_path", default=None, help="Existing model .zip for continue-highspeed")
    parser.add_argument("--total_timesteps", type=int, default=None)
    parser.add_argument("--reset_num_timesteps", action="store_true")
    parser.add_argument("--assist_gain", type=float, default=None, help="Override assist_gain for one run")
    parser.add_argument("--assist_gains", default=None, help="Comma-separated assist gains for sweep stages")
    parser.add_argument("--start_centerline_idx", type=int, default=None)
    parser.add_argument("--start_indices", default=None, help="Comma-separated raceline indices for future/manual sweeps")
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--out_dir", default=None)
    parser.add_argument("--print_every", type=int, default=None)
    parser.add_argument("--dry_run", action="store_true", help="Print commands and generated configs without running the low-level script")
    args = parser.parse_args()

    master = load_yaml(args.config)

    if args.stage == "sweep-highspeed":
        gains = parse_float_list(args.assist_gains, master.get("analysis", {}).get("highspeed_assist_gains", [0.25, 0.5, 0.75, 1.0]))
        for g in gains:
            run_one(args.config, master, "sweep-highspeed", args, assist_gain=g, start_idx=(0 if args.start_centerline_idx is None else args.start_centerline_idx))
        return
    if args.stage == "sweep-reserved":
        gains = parse_float_list(args.assist_gains, master.get("analysis", {}).get("reserved_assist_gains", [0.10, 0.25, 0.50]))
        for g in gains:
            run_one(args.config, master, "sweep-reserved", args, assist_gain=g, start_idx=(0 if args.start_centerline_idx is None else args.start_centerline_idx))
        return

    run_one(args.config, master, args.stage, args)


if __name__ == "__main__":
    main()
