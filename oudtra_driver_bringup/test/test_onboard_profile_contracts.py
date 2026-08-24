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


def test_local_planner_curvature_is_executable_in_both_profiles():
    profiles = []
    for filename in ("path_following_v2.yaml", "path_following_v2_sim.yaml"):
        document = yaml.safe_load(
            (REPOSITORY / "path_following_v2/config" / filename).read_text(
                encoding="utf-8"
            )
        )
        planner = document["local_trajectory_planner"]["ros__parameters"]
        follower = document["path_following_v2"]["ros__parameters"]
        required_steering_deg = math.degrees(
            math.atan(
                planner["curvature_safety_factor"]
                * math.tan(math.radians(planner["steering_max_deg"]))
            )
        )

        assert planner["curvature_safety_factor"] > 1.0
        assert follower["wheelbase_m"] == planner["wheelbase_m"]
        assert follower["steering_max_deg"] >= required_steering_deg
        profiles.append(
            (
                planner["curvature_safety_factor"],
                follower["steering_max_deg"],
            )
        )

    assert profiles[0] == profiles[1]
