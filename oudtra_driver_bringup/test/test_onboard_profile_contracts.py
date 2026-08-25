"""Contracts that keep onboard configuration independent from simulation."""

import csv
import math
from pathlib import Path

import yaml


REPOSITORY = Path(__file__).resolve().parents[2]


def _parameters(relative_path, node_name):
    document = yaml.safe_load(
        (REPOSITORY / relative_path).read_text(encoding="utf-8")
    )
    return document[node_name]["ros__parameters"]


def _closed_loop_length(csv_path):
    with csv_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    points = [(float(row["x"]), float(row["y"])) for row in rows]
    assert len(points) >= 3
    segments = [
        math.hypot(
            points[(index + 1) % len(points)][0] - point[0],
            points[(index + 1) % len(points)][1] - point[1],
        )
        for index, point in enumerate(points)
    ]
    return sum(segments), max(segments)


def test_onboard_topics_and_safety_gates_remain_hardware_specific():
    reactive = yaml.safe_load(
        (
            REPOSITORY
            / "reactive_control_v2/config/reactive_control_v2.yaml"
        ).read_text(encoding="utf-8")
    )
    upper = reactive["upper_corridor_follower"]["ros__parameters"]
    lower = reactive["lower_safety_controller"]["ros__parameters"]
    integration = _parameters(
        "oudtra_driver_bringup/config/full_stack.yaml",
        "lower_safety_controller",
    )
    arbitration = _parameters(
        "drive_arbitration_v2/config/drive_arbitration_v2.yaml",
        "drive_arbitrator",
    )

    assert upper["odom_topic"] == "/odom"
    assert lower["odom_topic"] == "/odom"
    assert lower["enable_wrong_way_recovery"] is False
    assert lower["enable_sim_reverse_swept_gate"] is False
    assert lower["enable_raceline_stall_handoff"] is False
    assert "enable_wrong_way_recovery" not in integration
    assert "enable_sim_reverse_swept_gate" not in integration
    assert arbitration["require_pf_health"] is True


def test_onboard_horizon_fits_the_default_closed_loop_raceline():
    generator = _parameters(
        "path_following_v2/config/path_following_v2.yaml", "path_generator"
    )
    raceline = (
        REPOSITORY
        / "centerline_tools/centerline_output/raceline_points_smooth.csv"
    )
    loop_length, longest_segment = _closed_loop_length(raceline)

    # path_generator publishes at most one lap and omits one loop-closing
    # segment, so every starting index must still be able to reach the target.
    assert generator["local_path_target_length_m"] <= loop_length - longest_segment


def test_onboard_speed_profiles_do_not_reference_simulator_artifacts():
    path = yaml.safe_load(
        (
            REPOSITORY / "path_following_v2/config/path_following_v2.yaml"
        ).read_text(encoding="utf-8")
    )
    follower = path["path_following_v2"]["ros__parameters"]
    rl = _parameters(
        "rl_speed_inference/config/rl_speed_inference.yaml", "ppo_speed_node"
    )

    assert follower["speed_policy_mode"] == 0
    assert not rl["model_path"].startswith("/sim_ws/")
    assert not rl["centerline_csv"].startswith("/sim_ws/")
    assert rl["odom_topic"] == "/pf/pose/odom"
    assert rl["assist_gain"] <= 0.10


def test_simulator_path_and_rl_speed_envelopes_remain_aligned():
    path = yaml.safe_load(
        (
            REPOSITORY / "path_following_v2/config/path_following_v2_sim.yaml"
        ).read_text(encoding="utf-8")
    )
    shared = path["/**"]["ros__parameters"]
    generator = path["path_generator"]["ros__parameters"]
    rl = _parameters(
        "rl_speed_inference/config/rl_speed_inference_sim.yaml", "ppo_speed_node"
    )

    assert shared["command_speed_max_mps"] == rl["command_speed_max_mps"]
    assert generator["rule_straight_speed_mps"] == rl["rule_straight_speed_mps"]


def test_local_planner_search_limits_match_in_both_profiles():
    profiles = []
    for filename in ("path_following_v2.yaml", "path_following_v2_sim.yaml"):
        document = yaml.safe_load(
            (REPOSITORY / "path_following_v2/config" / filename).read_text(
                encoding="utf-8"
            )
        )
        planner = document["local_trajectory_planner"]["ros__parameters"]
        generator = document["path_generator"]["ros__parameters"]
        follower = document["path_following_v2"]["ros__parameters"]

        assert planner["curvature_safety_factor"] > 1.0
        assert follower["wheelbase_m"] == planner["wheelbase_m"]
        assert planner["raw_path_topic"] == generator["local_path_topic"]
        assert planner["raceline_reference_topic"] == "/raceline_path"
        assert planner["require_raceline_reference"] is True
        assert planner["raceline_reference_match_tolerance_m"] > 0.0
        assert planner["require_map_clearance"] is True
        removed_centerline_contract = {
            "centerline_csv_path",
            "centerline_direction",
            "centerline_frame",
            "centerline_closed_loop",
            "require_centerline_reference",
        }
        assert removed_centerline_contract.isdisjoint(planner)
        profiles.append(
            (
                planner["max_lateral_shift_m"],
                planner["curvature_safety_factor"],
            )
        )

    assert profiles[0] == profiles[1] == (0.9, 2.7)


def test_runtime_launches_have_no_centerline_planning_arguments():
    launch_files = (
        "path_following_v2/launch/path_following_v2_launch.py",
        "path_following_v2/launch/path_following_v2_sim_launch.py",
        "oudtra_driver_bringup/launch/full_stack_onboard_launch.py",
        "oudtra_driver_bringup/launch/full_stack_sim_launch.py",
    )
    for relative_path in launch_files:
        source = (REPOSITORY / relative_path).read_text(encoding="utf-8")
        assert "centerline_csv_path" not in source
        assert "centerline_direction" not in source


def test_raceline_publisher_and_planner_share_one_reference():
    for suffix in ("", "_sim"):
        path_config = yaml.safe_load(
            (
                REPOSITORY
                / f"path_following_v2/config/path_following_v2{suffix}.yaml"
            ).read_text(encoding="utf-8")
        )
        publisher = _parameters(
            f"centerline_tools/config/raceline_publisher{suffix}.yaml",
            "raceline_publisher",
        )
        generator = path_config["path_generator"]["ros__parameters"]
        planner = path_config["local_trajectory_planner"]["ros__parameters"]

        assert publisher["path_topic"] == planner["raceline_reference_topic"]
        assert (
            publisher["waypoints_topic"]
            == generator["raceline_waypoints_topic"]
        )


def test_planning_freshness_windows_match_in_both_profiles():
    suffixes = ("", "_sim")
    for suffix in suffixes:
        path = yaml.safe_load(
            (
                REPOSITORY
                / f"path_following_v2/config/path_following_v2{suffix}.yaml"
            ).read_text(encoding="utf-8")
        )
        arbitration = _parameters(
            f"drive_arbitration_v2/config/drive_arbitration_v2{suffix}.yaml",
            "drive_arbitrator",
        )
        reactive = yaml.safe_load(
            (
                REPOSITORY
                / f"reactive_control_v2/config/reactive_control_v2{suffix}.yaml"
            ).read_text(encoding="utf-8")
        )

        planner = path["local_trajectory_planner"]["ros__parameters"]
        lower = reactive["lower_safety_controller"]["ros__parameters"]
        assert planner["raw_path_timeout_sec"] == 1.0
        assert planner["planning_heartbeat_timeout_sec"] == 1.0
        assert arbitration["path_status_timeout_sec"] == 1.0
        assert lower["wrong_way_heading_error_timeout_sec"] == 1.0
        assert planner["scan_timeout_sec"] == 0.30
        assert lower["scan_timeout_sec"] == 0.30
