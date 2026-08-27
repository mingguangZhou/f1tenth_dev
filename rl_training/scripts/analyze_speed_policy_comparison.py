#!/usr/bin/env python3
"""
Evaluate and compare the rule-based speed baseline against a trained residual PPO
speed policy along the raceline, using cumulative lap progress from the actual
start index.

Place this file in:
    /sim_ws/src/rl_training/scripts/analyze_speed_policy_comparison.py

Key fixes versus the previous analysis script:
    - Uses cumulative progress from the environment instead of raw raceline index.
    - Does not interpolate through unvisited track sections in the main plots.
    - Prints rollout progress in the terminal by default.
    - Produces CSV/PNG outputs that reveal whether the rollout covered a real lap.
"""

import argparse
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from stable_baselines3 import PPO

from rl_training.f110_speed_env import F110SpeedEnv
from rl_training.config_utils import parse_args_with_config, apply_start_centerline_idx


REWARD_TERM_KEYS = [
    "reward_progress",
    "reward_lap",
    "reward_early_finish",
    "penalty_tracking",
    "penalty_curvature_speed_section",
    "penalty_target_speed_smoothness",
    "penalty_residual_smoothness",
    "penalty_residual_excess",
    "penalty_assist_smoothness",
    "penalty_assist_excess",
    "penalty_time",
    "penalty_crash",
    "penalty_timeout",
]



@dataclass
class Summary:
    name: str
    executed_steps: int
    total_reward: float
    average_target_speed: float
    lap_completed: bool
    crashed: bool
    timeout: bool
    final_progress_idx: float
    final_progress_m: float
    final_lap_progress_ratio: float
    start_centerline_idx: int
    final_nearest_idx: int


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def load_raceline(csv_path: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    required = {"index", "x", "y", "yaw", "curvature", "curvature_abs"}
    missing = required.difference(df.columns)
    if missing:
        raise ValueError(f"Raceline CSV is missing columns: {sorted(missing)}")
    df = df.copy()
    df["index"] = df["index"].astype(int)
    return df.sort_values("index").reset_index(drop=True)


def load_corners(corner_csv: Optional[str]) -> Optional[pd.DataFrame]:
    if not corner_csv:
        return None
    if not os.path.exists(corner_csv):
        print(f"[WARN] Corner CSV not found: {corner_csv}. Corner labels disabled.")
        return None
    df = pd.read_csv(corner_csv)
    required = {"corner_id", "entrance_index", "apex_index", "exit_index"}
    missing = required.difference(df.columns)
    if missing:
        print(f"[WARN] Corner CSV missing {sorted(missing)}. Corner labels disabled.")
        return None
    return df


def circular_forward_delta(start_idx: int, idx: int, n_points: int) -> int:
    """Forward index distance on a closed loop from start_idx to idx."""
    return int((int(idx) - int(start_idx)) % int(n_points))


def create_env(args) -> F110SpeedEnv:
    return F110SpeedEnv(
        map_path=args.map_path,
        map_ext=args.map_ext,
        centerline_csv=args.centerline_csv,
        start_pose=(args.sx, args.sy, args.stheta),
        command_speed_min_mps=args.command_speed_min_mps,
        command_speed_max_mps=args.command_speed_max_mps,
        min_speed=args.min_speed,
        max_speed=args.max_speed,
        max_speed_index_delta=args.max_speed_index_delta,
        max_speed_delta_per_step_mps=args.max_speed_delta_per_step_mps,
        max_delta_speed_mps=args.max_delta_speed_mps,
        residual_output_mode=args.residual_output_mode,
        positive_assist_ratio=args.positive_assist_ratio,
        negative_assist_ratio=args.negative_assist_ratio,
        assist_gain=args.assist_gain,
        max_episode_steps=args.steps,
        target_lap_steps=args.target_lap_steps,
        curvature_gain=args.curvature_gain,
        residual_correction_scale=args.residual_correction_scale,
        rule_curvature_lookahead_points=args.rule_curvature_lookahead_points,
        model_curvature_short_points=args.model_curvature_short_points,
        model_curvature_mid_points=args.model_curvature_mid_points,
        model_curvature_long_points=args.model_curvature_long_points,
        rule_speed_curvature_preview_m=args.rule_speed_curvature_preview_m,
        model_curvature_short_preview_m=args.model_curvature_short_preview_m,
        model_curvature_mid_preview_m=args.model_curvature_mid_preview_m,
        model_curvature_long_preview_m=args.model_curvature_long_preview_m,
        random_start_along_centerline=args.random_start_along_centerline,
        random_start_min_index=args.random_start_min_index,
        random_start_max_index=args.random_start_max_index,
        start_lateral_noise_std=args.start_lateral_noise_std,
        start_lateral_noise_max=args.start_lateral_noise_max,
        start_yaw_noise_std=args.start_yaw_noise_std,
        start_yaw_noise_max=args.start_yaw_noise_max,
        start_xy_noise_std=args.start_xy_noise_std,
        start_xy_noise_max=args.start_xy_noise_max,
        obs_cte_noise_std=args.obs_cte_noise_std,
        obs_heading_noise_std=args.obs_heading_noise_std,
        obs_speed_noise_std=args.obs_speed_noise_std,

        rule_min_speed_mps=args.rule_min_speed_mps,
        rule_max_speed_mps=args.rule_max_speed_mps,
        rule_curve_min_speed_mps=args.rule_curve_min_speed_mps,
        rule_straight_speed_mps=args.rule_straight_speed_mps,
        rule_speed_curvature_gain=args.rule_speed_curvature_gain,
        enable_bad_tracking_termination=args.enable_bad_tracking_termination,
        bad_tracking_min_steps=args.bad_tracking_min_steps,
        bad_tracking_cte_threshold=args.bad_tracking_cte_threshold,
        bad_tracking_heading_threshold=args.bad_tracking_heading_threshold,
        crash_penalty_value=args.crash_penalty_value,
        timeout_penalty_value=args.timeout_penalty_value,
        target_speed_smoothness_weight=args.target_speed_smoothness_weight,
        reward_curvature_section_start_points=args.reward_curvature_section_start_points,
        reward_curvature_section_end_points=args.reward_curvature_section_end_points,
        reward_curvature_section_start_m=args.reward_curvature_section_start_m,
        reward_curvature_section_end_m=args.reward_curvature_section_end_m,
        curvature_speed_section_weight=args.curvature_speed_section_weight,
        residual_smoothness_weight=args.residual_smoothness_weight,
        residual_free_band_mps=args.residual_free_band_mps,
        residual_excess_weight=args.residual_excess_weight,
        assist_smoothness_weight=args.assist_smoothness_weight,
        assist_free_band=args.assist_free_band,
        assist_excess_weight=args.assist_excess_weight,
        enable_rl_gate=args.enable_rl_gate,
        rl_gate_enable_cte=args.rl_gate_enable_cte,
        rl_gate_enable_heading=args.rl_gate_enable_heading,
        rl_gate_disable_cte=args.rl_gate_disable_cte,
        rl_gate_disable_heading=args.rl_gate_disable_heading,
        rl_gate_enable_count=args.rl_gate_enable_count,
        rl_gate_disable_count=args.rl_gate_disable_count,
        rl_gate_fade_in_step=args.rl_gate_fade_in_step,
        rl_gate_fade_out_step=args.rl_gate_fade_out_step,
        random_seed=args.random_seed,
        wheelbase_m=args.wheelbase_m,
        steering_max_deg=args.steering_max_deg,
        fixed_steering_lookahead_m=args.fixed_steering_lookahead_m,
        use_speed_dependent_steering_lookahead=args.use_speed_dependent_steering_lookahead,
        steering_min_lookahead_m=args.steering_min_lookahead_m,
        steering_max_lookahead_m=args.steering_max_lookahead_m,
        steering_lookahead_speed_gain=args.steering_lookahead_speed_gain,
        min_forward_point_x_m=args.min_forward_point_x_m,
        use_speed_dependent_lookahead=True,
    )


def rollout_policy(args, name: str, model: Optional[PPO]) -> Tuple[pd.DataFrame, Summary]:
    env = create_env(args)
    obs = env.reset()

    rows: List[Dict] = []
    total_reward = 0.0
    speed_sum = 0.0
    n_points = len(env.centerline)
    spacing = float(env.average_waypoint_spacing)

    print(f"\n[{name}] rollout started: steps={args.steps}, start_idx={env.start_centerline_idx}")

    for step in range(args.steps):
        if model is None:
            # Residual environment: action 0.0 follows the curvature-rule baseline.
            action = np.array([0.0], dtype=np.float32)
        else:
            action_raw, _ = model.predict(obs, deterministic=True)
            action = np.array([np.clip(float(action_raw[0]), -1.0, 1.0)], dtype=np.float32)

        obs, reward, done, info = env.step(action)
        _, nearest_idx_helper = env._get_rl_observation_with_index()
        car_x, car_y, car_yaw, car_speed = env._get_car_state()

        nearest_idx = int(info.get("nearest_idx", nearest_idx_helper))
        start_idx = int(info.get("start_centerline_idx", env.start_centerline_idx))

        # Prefer the environment's cumulative progress. If the current env does
        # not expose it yet, fall back to circular progress from start index.
        cumulative_progress_idx = float(
            info.get("cumulative_progress_idx", circular_forward_delta(start_idx, nearest_idx, n_points))
        )
        cumulative_progress_m = float(info.get("cumulative_progress_m", cumulative_progress_idx * spacing))
        lap_progress_ratio = float(info.get("lap_progress_ratio", cumulative_progress_idx / max(n_points, 1)))

        target_speed = float(info.get("target_speed_mps", 0.0))
        total_reward += float(reward)
        speed_sum += target_speed

        rows.append(
            {
                "policy": name,
                "step": step,
                "nearest_idx": nearest_idx,
                "start_centerline_idx": start_idx,
                "cumulative_progress_idx": cumulative_progress_idx,
                "cumulative_progress_m": cumulative_progress_m,
                "lap_progress_ratio": lap_progress_ratio,
                "x": car_x,
                "y": car_y,
                "yaw": car_yaw,
                "actual_speed_mps": car_speed,
                "actual_speed_mps_obs": float(obs[0]),
                "actual_speed_index": float(info.get("executed_speed_index", np.nan)),
                "rule_speed_mps_obs": float(obs[1]),
                "cte": float(obs[2]),
                "heading_error": float(obs[3]),
                "curv_short_abs": float(obs[4]),
                "curv_mid_abs": float(obs[5]) if len(obs) >= 6 else np.nan,
                "curv_long_abs": float(obs[6]) if len(obs) >= 7 else np.nan,
                "upcoming_curvature_abs": float(obs[5]) if len(obs) >= 6 else float(obs[4]),
                "previous_delta_speed_mps_obs": float(obs[7]) if len(obs) >= 8 else np.nan,
                "correction_action": float(info.get("correction_action", action[0])),
                "raw_delta_speed_mps": float(info.get("raw_delta_speed_mps", info.get("delta_speed_mps", np.nan))),
                "raw_assist_ratio": float(info.get("raw_assist_ratio", np.nan)),
                "assist_ratio": float(info.get("assist_ratio", np.nan)),
                "assist_gain": float(info.get("assist_gain", np.nan)),
                "rl_gate_enabled": bool(info.get("rl_gate_enabled", True)),
                "rl_gate_scale": float(info.get("rl_gate_scale", 1.0)),
                "rl_gate_good_count": int(info.get("rl_gate_good_count", 0)),
                "rl_gate_bad_count": int(info.get("rl_gate_bad_count", 0)),
                "delta_speed_mps": float(info.get("delta_speed_mps", np.nan)),
                "rule_speed_mps": float(info.get("rule_speed_mps", np.nan)),
                "rule_speed_index": float(info.get("rule_speed_index", np.nan)),
                "speed_index_correction": float(info.get("speed_index_correction", np.nan)),
                "requested_speed_mps": float(info.get("requested_speed_mps", np.nan)),
                "requested_speed_index": float(info.get("requested_speed_index", np.nan)),
                "executed_speed_index": float(info.get("executed_speed_index", np.nan)),
                "target_speed_mps": target_speed,
                "steering_rad": float(info.get("steering_rad", np.nan)),
                "reward": float(reward),
                "reward_progress": float(info.get("reward_progress", 0.0)),
                "reward_lap": float(info.get("reward_lap", 0.0)),
                "reward_early_finish": float(info.get("reward_early_finish", 0.0)),
                "penalty_tracking": float(info.get("penalty_tracking", 0.0)),
                "penalty_curvature_speed_section": float(info.get("penalty_curvature_speed_section", 0.0)),
                "penalty_target_speed_smoothness": float(info.get("penalty_target_speed_smoothness", 0.0)),
                "penalty_residual_smoothness": float(info.get("penalty_residual_smoothness", 0.0)),
                "penalty_residual_excess": float(info.get("penalty_residual_excess", 0.0)),
                "residual_excess_mps": float(info.get("residual_excess_mps", 0.0)),
                "penalty_time": float(info.get("penalty_time", 0.0)),
                "penalty_crash": float(info.get("penalty_crash", 0.0)),
                "penalty_timeout": float(info.get("penalty_timeout", 0.0)),
                "reward_curvature_section_abs": float(info.get("reward_curvature_section_abs", np.nan)),
                "lap_completed": bool(env.lap_completed),
                "crashed": bool(env.crashed),
                "timeout": bool(env.timeout),
            }
        )

        if args.print_every > 0 and (step % args.print_every == 0 or done):
            print(
                f"[{name}] step={step:05d} idx={nearest_idx:05d} "
                f"prog={lap_progress_ratio:5.1%} "
                f"action={rows[-1]['correction_action']: .3f} "
                f"raw_delta={rows[-1]['raw_delta_speed_mps']: .3f} "
                f"gate={rows[-1]['rl_gate_scale']: .2f} "
                f"delta_v={rows[-1]['delta_speed_mps']: .3f} "
                f"rule_v={rows[-1]['rule_speed_mps']: .3f} "
                f"v={target_speed: .3f} cte={obs[2]: .3f} "
                f"curv_s={obs[4]: .3f} curv_m={obs[5]: .3f} curv_l={obs[6]: .3f} r={reward: .3f} "
                f"done={done}"
            )

        if done:
            break

    df = pd.DataFrame(rows)
    if df.empty:
        raise RuntimeError(f"{name} rollout produced no samples")

    summary = Summary(
        name=name,
        executed_steps=len(df),
        total_reward=total_reward,
        average_target_speed=speed_sum / max(len(df), 1),
        lap_completed=bool(env.lap_completed),
        crashed=bool(env.crashed),
        timeout=bool(env.timeout),
        final_progress_idx=float(df["cumulative_progress_idx"].iloc[-1]),
        final_progress_m=float(df["cumulative_progress_m"].iloc[-1]),
        final_lap_progress_ratio=float(df["lap_progress_ratio"].iloc[-1]),
        start_centerline_idx=int(df["start_centerline_idx"].iloc[0]),
        final_nearest_idx=int(df["nearest_idx"].iloc[-1]),
    )

    print(
        f"[{name}] summary: steps={summary.executed_steps}, "
        f"reward={summary.total_reward:.3f}, avg_speed={summary.average_target_speed:.3f}, "
        f"lap={summary.lap_completed}, crash={summary.crashed}, timeout={summary.timeout}, "
        f"progress={summary.final_lap_progress_ratio:.1%}"
    )
    return df, summary


def interpolation_on_progress(df: pd.DataFrame, grid_m: np.ndarray, column: str) -> np.ndarray:
    if column not in df.columns or df.empty:
        return np.full_like(grid_m, np.nan, dtype=float)
    work = df[["cumulative_progress_m", column]].dropna().sort_values("cumulative_progress_m")
    if work.empty:
        return np.full_like(grid_m, np.nan, dtype=float)
    # Deduplicate progress values for np.interp.
    work = work.groupby("cumulative_progress_m", as_index=False)[column].mean()
    x = work["cumulative_progress_m"].to_numpy(dtype=float)
    y = work[column].to_numpy(dtype=float)
    if len(x) < 2:
        return np.full_like(grid_m, np.nan, dtype=float)
    out = np.interp(grid_m, x, y)
    out[grid_m < x[0]] = np.nan
    out[grid_m > x[-1]] = np.nan
    return out


def aligned_comparison(rule_df: pd.DataFrame, model_df: pd.DataFrame, raceline: pd.DataFrame) -> pd.DataFrame:
    spacing = estimate_raceline_spacing(raceline)
    total_len_m = len(raceline) * spacing
    max_progress = max(rule_df["cumulative_progress_m"].max(), model_df["cumulative_progress_m"].max())
    grid_m = np.linspace(0.0, min(max_progress, total_len_m), num=max(1000, len(raceline)))

    aligned = pd.DataFrame({"progress_m": grid_m, "progress_idx_est": grid_m / max(spacing, 1e-6)})
    columns = [
        "target_speed_mps",
        "actual_speed_mps",
        "requested_speed_index",
        "executed_speed_index",
        "rule_speed_mps",
        "rule_speed_index",
        "delta_speed_mps",
        "speed_index_correction",
        "correction_action",
        "cte",
        "heading_error",
        "curv_short_abs",
        "curv_mid_abs",
        "curv_long_abs",
        "upcoming_curvature_abs",
        "reward",
    ]
    for prefix, df in [("rule", rule_df), ("model", model_df)]:
        for col in columns:
            aligned[f"{prefix}_{col}"] = interpolation_on_progress(df, grid_m, col)

    if "model_target_speed_mps" in aligned and "rule_target_speed_mps" in aligned:
        aligned["delta_target_speed_mps"] = aligned["model_target_speed_mps"] - aligned["rule_target_speed_mps"]
    if "model_executed_speed_index" in aligned and "rule_executed_speed_index" in aligned:
        aligned["delta_executed_speed_index"] = aligned["model_executed_speed_index"] - aligned["rule_executed_speed_index"]
    if "model_requested_speed_index" in aligned and "rule_requested_speed_index" in aligned:
        aligned["delta_requested_speed_index"] = aligned["model_requested_speed_index"] - aligned["rule_requested_speed_index"]
    return aligned


def estimate_raceline_spacing(raceline: pd.DataFrame) -> float:
    if len(raceline) < 2:
        return 0.05
    x = raceline["x"].to_numpy(dtype=float)
    y = raceline["y"].to_numpy(dtype=float)
    dx = np.roll(x, -1) - x
    dy = np.roll(y, -1) - y
    return float(np.nanmean(np.hypot(dx, dy)))


def corner_progress_rows(corners: Optional[pd.DataFrame], start_idx: int, n_points: int, spacing: float) -> List[Dict]:
    if corners is None:
        return []
    rows: List[Dict] = []
    for _, row in corners.iterrows():
        cid = int(row["corner_id"])
        e_idx = int(row["entrance_index"])
        a_idx = int(row["apex_index"])
        x_idx = int(row["exit_index"])
        e = circular_forward_delta(start_idx, e_idx, n_points)
        a = circular_forward_delta(start_idx, a_idx, n_points)
        x = circular_forward_delta(start_idx, x_idx, n_points)
        rows.append({
            "corner_id": cid,
            "turn_direction": row.get("turn_direction", ""),
            "entrance_index": e_idx,
            "apex_index": a_idx,
            "exit_index": x_idx,
            "entrance_progress_idx": e,
            "apex_progress_idx": a,
            "exit_progress_idx": x,
            "entrance_progress_m": e * spacing,
            "apex_progress_m": a * spacing,
            "exit_progress_m": x * spacing,
        })
    return rows


def add_corner_spans_progress(ax, corner_rows: List[Dict], total_len_m: float) -> None:
    if not corner_rows:
        return
    ymin, ymax = ax.get_ylim()
    for c in corner_rows:
        cid = int(c["corner_id"])
        e = float(c["entrance_progress_m"])
        a = float(c["apex_progress_m"])
        x = float(c["exit_progress_m"])
        if e <= x:
            spans = [(e, x)]
        else:
            spans = [(e, total_len_m), (0.0, x)]
        for s0, s1 in spans:
            ax.axvspan(s0, s1, alpha=0.08)
        ax.axvline(a, linestyle="--", linewidth=0.8, alpha=0.45)
        if 0.0 <= a <= total_len_m:
            ax.text(a, ymax, f"C{cid}", fontsize=8, rotation=90, va="top", ha="center")
    ax.set_ylim(ymin, ymax)


def save_speed_plot(rule_df: pd.DataFrame, model_df: pd.DataFrame, corner_rows: List[Dict], total_len_m: float, out_dir: str) -> None:
    fig, ax1 = plt.subplots(figsize=(16, 6))
    ax1.plot(rule_df["cumulative_progress_m"], rule_df["target_speed_mps"], label="Rule-based target speed", linewidth=1.8)
    ax1.plot(model_df["cumulative_progress_m"], model_df["target_speed_mps"], label="Model target speed", linewidth=1.8)
    ax1.set_xlabel("Cumulative progress from start [m]")
    ax1.set_ylabel("Target speed [m/s]")
    ax1.grid(True, alpha=0.3)
    add_corner_spans_progress(ax1, corner_rows, total_len_m)

    ax2 = ax1.twinx()
    ax2.plot(rule_df["cumulative_progress_m"], rule_df["curv_short_abs"], label="Rule curv short", linestyle=":", linewidth=1.0, alpha=0.65)
    ax2.plot(rule_df["cumulative_progress_m"], rule_df["curv_mid_abs"], label="Rule curv mid", linestyle="--", linewidth=0.8, alpha=0.45)
    ax2.plot(rule_df["cumulative_progress_m"], rule_df["curv_long_abs"], label="Rule curv long", linestyle="-.", linewidth=0.8, alpha=0.35)
    ax2.set_ylabel("|curvature| [1/m]")

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "speed_vs_cumulative_progress.png"), dpi=160)
    plt.close(fig)


def save_residual_plot(model_df: pd.DataFrame, corner_rows: List[Dict], total_len_m: float, out_dir: str) -> None:
    fig, ax = plt.subplots(figsize=(16, 5))
    x = model_df["cumulative_progress_m"]
    ax.plot(x, model_df["delta_speed_mps"], label="Model delta_speed_mps", linewidth=1.5)
    ax.plot(x, model_df["correction_action"], label="Model correction_action", linewidth=1.0, alpha=0.7)
    ax.axhline(0.0, color="black", linewidth=0.8)
    ax.set_xlabel("Cumulative progress from start [m]")
    ax.set_ylabel("Physical residual [m/s] / correction action")
    ax.grid(True, alpha=0.3)
    add_corner_spans_progress(ax, corner_rows, total_len_m)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "residual_vs_cumulative_progress.png"), dpi=160)
    plt.close(fig)


def save_tracking_plot(rule_df: pd.DataFrame, model_df: pd.DataFrame, corner_rows: List[Dict], total_len_m: float, out_dir: str) -> None:
    fig, ax = plt.subplots(figsize=(16, 5))
    ax.plot(rule_df["cumulative_progress_m"], rule_df["cte"], label="Rule CTE", linewidth=1.1)
    ax.plot(model_df["cumulative_progress_m"], model_df["cte"], label="Model CTE", linewidth=1.1)
    ax.plot(rule_df["cumulative_progress_m"], rule_df["heading_error"], label="Rule heading error", linewidth=0.9, alpha=0.75)
    ax.plot(model_df["cumulative_progress_m"], model_df["heading_error"], label="Model heading error", linewidth=0.9, alpha=0.75)
    ax.axhline(0.0, color="black", linewidth=0.8)
    ax.set_xlabel("Cumulative progress from start [m]")
    ax.set_ylabel("Tracking error [m or rad]")
    ax.grid(True, alpha=0.3)
    add_corner_spans_progress(ax, corner_rows, total_len_m)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "tracking_errors_vs_cumulative_progress.png"), dpi=160)
    plt.close(fig)


def save_xy_plot(raceline: pd.DataFrame, rule_df: pd.DataFrame, model_df: pd.DataFrame, out_dir: str) -> None:
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.plot(raceline["x"], raceline["y"], color="black", linewidth=0.8, label="Raceline")
    sc1 = ax.scatter(rule_df["x"], rule_df["y"], c=rule_df["target_speed_mps"], s=5, alpha=0.55, label="Rule samples")
    ax.scatter(model_df["x"], model_df["y"], c=model_df["target_speed_mps"], s=5, alpha=0.55, marker="x", label="Model samples")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Visited track samples colored by target speed")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.colorbar(sc1, ax=ax, label="Target speed [m/s]")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "xy_speed_trace.png"), dpi=160)
    plt.close(fig)


def nearest_record_by_progress(df: pd.DataFrame, progress_m: float, window_m: float = 2.0) -> Optional[pd.Series]:
    if df.empty:
        return None
    d = np.abs(df["cumulative_progress_m"].to_numpy(dtype=float) - float(progress_m))
    best_i = int(np.argmin(d))
    if d[best_i] > window_m:
        return None
    return df.iloc[best_i]


def make_corner_summary(rule_df: pd.DataFrame, model_df: pd.DataFrame, corner_rows: List[Dict]) -> pd.DataFrame:
    rows = []
    for c in corner_rows:
        cid = int(c["corner_id"])
        for role in ["entrance", "apex", "exit"]:
            progress_m = float(c[f"{role}_progress_m"])
            rr = nearest_record_by_progress(rule_df, progress_m)
            mr = nearest_record_by_progress(model_df, progress_m)
            rows.append(
                {
                    "corner_id": cid,
                    "role": role,
                    "progress_m": progress_m,
                    "raceline_index": int(c[f"{role}_index"]),
                    "turn_direction": c.get("turn_direction", ""),
                    "rule_target_speed_mps": np.nan if rr is None else rr["target_speed_mps"],
                    "model_target_speed_mps": np.nan if mr is None else mr["target_speed_mps"],
                    "rule_executed_speed_index": np.nan if rr is None else rr["executed_speed_index"],
                    "model_executed_speed_index": np.nan if mr is None else mr["executed_speed_index"],
                    "model_correction_action": np.nan if mr is None else mr["correction_action"],
                    "model_delta_speed_mps": np.nan if mr is None else mr["delta_speed_mps"],
                    "model_speed_index_correction": np.nan if mr is None else mr["speed_index_correction"],
                }
            )
    out = pd.DataFrame(rows)
    if not out.empty:
        out["delta_target_speed_mps"] = out["model_target_speed_mps"] - out["rule_target_speed_mps"]
    return out


def save_corner_table_plot(corner_summary: pd.DataFrame, out_dir: str) -> None:
    if corner_summary.empty:
        return
    labels = [f"C{r.corner_id}-{r.role[0].upper()}" for r in corner_summary.itertuples()]
    x = np.arange(len(corner_summary))
    width = 0.38
    fig, ax = plt.subplots(figsize=(max(12, len(labels) * 0.45), 5))
    ax.bar(x - width / 2, corner_summary["rule_target_speed_mps"], width, label="Rule")
    ax.bar(x + width / 2, corner_summary["model_target_speed_mps"], width, label="Model")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=70, ha="right")
    ax.set_ylabel("Target speed [m/s]")
    ax.set_title("Entrance / Apex / Exit speed comparison")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "corner_entrance_apex_exit_speeds.png"), dpi=160)
    plt.close(fig)


def write_reward_breakdown(rule_df: pd.DataFrame, model_df: pd.DataFrame, out_dir: str) -> None:
    rows = []
    for name, df in [("rule", rule_df), ("model", model_df)]:
        row = {"name": name}
        for key in REWARD_TERM_KEYS:
            row[key] = float(df[key].sum()) if key in df.columns else 0.0
        row["reward_total_from_steps"] = float(df["reward"].sum()) if "reward" in df.columns else 0.0
        row["mean_curvature_section_abs"] = float(df["reward_curvature_section_abs"].mean()) if "reward_curvature_section_abs" in df.columns else float("nan")
        rows.append(row)
    out = pd.DataFrame(rows)
    out.to_csv(os.path.join(out_dir, "reward_breakdown_summary.csv"), index=False)
    print("\n=== Reward breakdown summary ===")
    print(out.to_string(index=False))


def write_summary(summaries: List[Summary], out_dir: str) -> None:
    df = pd.DataFrame([s.__dict__ for s in summaries])
    df.to_csv(os.path.join(out_dir, "evaluation_summary.csv"), index=False)
    print("\n=== Evaluation summaries ===")
    print(df.to_string(index=False))


def parse_args():
    p = argparse.ArgumentParser(description="Compare rule-based and residual PPO speed policies along a raceline.")
    p.add_argument("--model_path", default=None)
    p.add_argument("--map_path", default=None)
    p.add_argument("--map_ext", default=None)
    p.add_argument("--centerline_csv", default=None, help="Usually raceline_points_smooth.csv")
    p.add_argument("--corner_csv", default=None, help="Optional corner_key_points_edited.csv for corner labels/tables")
    p.add_argument("--sx", type=float, default=None)
    p.add_argument("--sy", type=float, default=None)
    p.add_argument("--stheta", type=float, default=None)
    p.add_argument("--start_centerline_idx", type=int, default=-1, help="Optional raceline CSV index used to set sx/sy/stheta for same-start tests")
    p.add_argument("--steps", type=int, default=12000)
    p.add_argument("--min_speed", type=float, default=0.8, help="Legacy alias for command_speed_min_mps")
    p.add_argument("--max_speed", type=float, default=5.0, help="Legacy alias for command_speed_max_mps")
    p.add_argument("--command_speed_min_mps", type=float, default=None)
    p.add_argument("--command_speed_max_mps", type=float, default=None)
    p.add_argument("--max_speed_index_delta", type=float, default=0.05, help="Deprecated compatibility arg")
    p.add_argument("--max_speed_delta_per_step_mps", type=float, default=0.10)
    p.add_argument("--max_delta_speed_mps", type=float, default=0.30)
    p.add_argument("--residual_output_mode", default="physical_mps", choices=["physical_mps", "speed_ratio"])
    p.add_argument("--positive_assist_ratio", type=float, default=0.667)
    p.add_argument("--negative_assist_ratio", type=float, default=0.50)
    p.add_argument("--assist_gain", type=float, default=1.0)
    p.add_argument("--curvature_gain", type=float, default=2.0)
    p.add_argument("--residual_correction_scale", type=float, default=0.10, help="Deprecated compatibility arg")
    p.add_argument("--rule_curvature_lookahead_points", type=int, default=3)
    p.add_argument("--model_curvature_short_points", type=int, default=10)
    p.add_argument("--model_curvature_mid_points", type=int, default=40)
    p.add_argument("--model_curvature_long_points", type=int, default=80)
    p.add_argument("--rule_speed_curvature_preview_m", type=float, default=None)
    p.add_argument("--model_curvature_short_preview_m", type=float, default=None)
    p.add_argument("--model_curvature_mid_preview_m", type=float, default=None)
    p.add_argument("--model_curvature_long_preview_m", type=float, default=None)
    p.add_argument("--target_lap_steps", type=int, default=12000)
    p.add_argument("--random_start_along_centerline", action="store_true")
    p.add_argument("--random_start_min_index", type=int, default=-1)
    p.add_argument("--random_start_max_index", type=int, default=-1)
    p.add_argument("--start_lateral_noise_std", type=float, default=0.0)
    p.add_argument("--start_lateral_noise_max", type=float, default=0.05)
    p.add_argument("--start_yaw_noise_std", type=float, default=0.0)
    p.add_argument("--start_yaw_noise_max", type=float, default=0.05)
    p.add_argument("--start_xy_noise_std", type=float, default=0.0)
    p.add_argument("--start_xy_noise_max", type=float, default=0.05)
    p.add_argument("--obs_cte_noise_std", type=float, default=0.0)
    p.add_argument("--obs_heading_noise_std", type=float, default=0.0)
    p.add_argument("--obs_speed_noise_std", type=float, default=0.0)
    # High-speed residual training: separate the conservative rule-based
    # reference range from the physical command clamp.
    p.add_argument("--rule_min_speed_mps", type=float, default=None, help="Legacy alias for rule_curve_min_speed_mps")
    p.add_argument("--rule_max_speed_mps", type=float, default=None, help="Legacy alias for rule_straight_speed_mps")
    p.add_argument("--rule_curve_min_speed_mps", type=float, default=None)
    p.add_argument("--rule_straight_speed_mps", type=float, default=None)
    p.add_argument("--rule_speed_curvature_gain", type=float, default=None)

    # Optional early failure when the car has effectively lost the raceline.
    p.add_argument("--enable_bad_tracking_termination", action="store_true")
    p.add_argument("--bad_tracking_min_steps", type=int, default=50)
    p.add_argument("--bad_tracking_cte_threshold", type=float, default=0.75)
    p.add_argument("--bad_tracking_heading_threshold", type=float, default=0.90)

    # Reward shaping constants exposed for high-speed experiments.
    p.add_argument("--crash_penalty_value", type=float, default=1000.0)
    p.add_argument("--timeout_penalty_value", type=float, default=500.0)
    p.add_argument("--target_speed_smoothness_weight", type=float, default=0.04)
    p.add_argument("--reward_curvature_section_start_points", type=int, default=2)
    p.add_argument("--reward_curvature_section_end_points", type=int, default=40)
    p.add_argument("--reward_curvature_section_start_m", type=float, default=None)
    p.add_argument("--reward_curvature_section_end_m", type=float, default=None)
    p.add_argument("--curvature_speed_section_weight", type=float, default=0.006)
    p.add_argument("--residual_smoothness_weight", type=float, default=0.08)
    p.add_argument("--residual_free_band_mps", type=float, default=0.8)
    p.add_argument("--residual_excess_weight", type=float, default=0.05)
    p.add_argument("--assist_smoothness_weight", type=float, default=0.08)
    p.add_argument("--assist_free_band", type=float, default=0.15)
    p.add_argument("--assist_excess_weight", type=float, default=0.05)
    # Optional runtime/inference safety gate for the learned residual.
    p.add_argument("--enable_rl_gate", action="store_true")
    p.add_argument("--rl_gate_enable_cte", type=float, default=0.25)
    p.add_argument("--rl_gate_enable_heading", type=float, default=0.20)
    p.add_argument("--rl_gate_disable_cte", type=float, default=0.45)
    p.add_argument("--rl_gate_disable_heading", type=float, default=0.35)
    p.add_argument("--rl_gate_enable_count", type=int, default=10)
    p.add_argument("--rl_gate_disable_count", type=int, default=3)
    p.add_argument("--rl_gate_fade_in_step", type=float, default=0.05)
    p.add_argument("--rl_gate_fade_out_step", type=float, default=0.10)

    p.add_argument("--wheelbase_m", type=float, default=None)
    p.add_argument("--steering_max_deg", type=float, default=None)
    p.add_argument("--fixed_steering_lookahead_m", type=float, default=None)
    p.add_argument("--use_speed_dependent_steering_lookahead", action="store_true", default=None)
    p.add_argument("--steering_min_lookahead_m", type=float, default=None)
    p.add_argument("--steering_max_lookahead_m", type=float, default=None)
    p.add_argument("--steering_lookahead_speed_gain", type=float, default=None)
    p.add_argument("--min_forward_point_x_m", type=float, default=0.05)

    p.add_argument("--random_seed", type=int, default=None)
    p.add_argument("--out_dir", default="speed_policy_analysis")
    p.add_argument("--print_every", type=int, default=200, help="0 disables rollout progress printing")
    args = parse_args_with_config(
        p,
        required_keys=["model_path", "map_path", "map_ext", "centerline_csv", "sx", "sy", "stheta"],
    )
    return apply_start_centerline_idx(args)


def main():
    args = parse_args()
    ensure_dir(args.out_dir)

    raceline = load_raceline(args.centerline_csv)
    spacing = estimate_raceline_spacing(raceline)
    total_len_m = len(raceline) * spacing

    corners = load_corners(args.corner_csv)

    print("=== Speed policy analysis ===")
    print(f"Raceline points: {len(raceline)}")
    print(f"Estimated raceline length: {total_len_m:.2f} m")
    print(f"Output directory: {os.path.abspath(args.out_dir)}")
    print("Loading PPO model:", args.model_path)
    model = PPO.load(args.model_path)

    print("\nRunning rule-based baseline rollout...")
    rule_df, rule_summary = rollout_policy(args, "rule", model=None)

    print("\nRunning PPO model rollout...")
    model_df, model_summary = rollout_policy(args, "model", model=model)

    start_idx = int(rule_df["start_centerline_idx"].iloc[0])
    corner_rows = corner_progress_rows(corners, start_idx, len(raceline), spacing)
    if corner_rows:
        pd.DataFrame(corner_rows).to_csv(os.path.join(args.out_dir, "corner_progress_reference.csv"), index=False)
        print(f"Loaded {len(corner_rows)} corner definitions, converted to progress from start_idx={start_idx}.")
    else:
        print("No corner definitions used.")

    rule_df.to_csv(os.path.join(args.out_dir, "rule_timeseries.csv"), index=False)
    model_df.to_csv(os.path.join(args.out_dir, "model_timeseries.csv"), index=False)

    aligned = aligned_comparison(rule_df, model_df, raceline)
    aligned.to_csv(os.path.join(args.out_dir, "aligned_by_cumulative_progress.csv"), index=False)

    corner_summary = make_corner_summary(rule_df, model_df, corner_rows)
    if not corner_summary.empty:
        corner_summary.to_csv(os.path.join(args.out_dir, "corner_speed_summary.csv"), index=False)
        print("\n=== Corner speed summary preview ===")
        print(corner_summary.head(20).to_string(index=False))

    save_speed_plot(rule_df, model_df, corner_rows, total_len_m, args.out_dir)
    save_residual_plot(model_df, corner_rows, total_len_m, args.out_dir)
    save_tracking_plot(rule_df, model_df, corner_rows, total_len_m, args.out_dir)
    save_xy_plot(raceline, rule_df, model_df, args.out_dir)
    save_corner_table_plot(corner_summary, args.out_dir)
    write_reward_breakdown(rule_df, model_df, args.out_dir)
    write_summary([rule_summary, model_summary], args.out_dir)

    print("\nWrote analysis outputs to:", os.path.abspath(args.out_dir))
    print("Key files:")
    for f in [
        "evaluation_summary.csv",
        "reward_breakdown_summary.csv",
        "rule_timeseries.csv",
        "model_timeseries.csv",
        "aligned_by_cumulative_progress.csv",
        "corner_progress_reference.csv",
        "corner_speed_summary.csv",
        "speed_vs_cumulative_progress.png",
        "residual_vs_cumulative_progress.png",
        "tracking_errors_vs_cumulative_progress.png",
        "corner_entrance_apex_exit_speeds.png",
        "xy_speed_trace.png",
    ]:
        path = os.path.join(args.out_dir, f)
        if os.path.exists(path):
            print("  -", path)


if __name__ == "__main__":
    main()