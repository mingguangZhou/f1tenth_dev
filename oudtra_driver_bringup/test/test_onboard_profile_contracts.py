"""Contracts for canonical behavior plus sparse platform adapters."""

import ast
from pathlib import Path

import yaml


REPOSITORY = Path(__file__).resolve().parents[2]


def _yaml(relative_path):
    return yaml.safe_load(
        (REPOSITORY / relative_path).read_text(encoding="utf-8")
    ) or {}


def _parameters(document, node_name):
    return document[node_name]["ros__parameters"]


def _overlay_keys(document):
    return {
        f"{node_name}.{parameter_name}"
        for node_name, node in document.items()
        for parameter_name in node.get("ros__parameters", {})
    }


def test_race_ready_parameter_profile_is_canonical():
    path = _yaml("path_following_v2/config/path_following_v2.yaml")
    shared = _parameters(path, "/**")
    generator = _parameters(path, "path_generator")
    planner = _parameters(path, "local_trajectory_planner")
    reactive = _yaml("reactive_control_v2/config/reactive_control_v2.yaml")
    upper = _parameters(reactive, "upper_corridor_follower")
    lower = _parameters(reactive, "lower_safety_controller")

    assert shared["command_speed_max_mps"] == 9.0
    assert generator["rule_curve_min_speed_mps"] == 0.9
    assert generator["rule_straight_speed_mps"] == 4.5
    assert planner["transform_timeout_sec"] == 0.40
    assert planner["planning_distance_m"] == 10.0
    assert planner["curvature_safety_factor"] == 1.2
    assert planner["avoidance_speed_cap_mps"] == 3.15
    assert planner["recovery_speed_cap_mps"] == 3.6
    assert planner["replan_pending_speed_cap_mps"] == 2.25
    assert planner["maneuver_lateral_acceleration_limit_mps2"] == 3.24
    assert planner["yield_max_speed_mps"] == 2.25
    assert upper["forward_max_m"] == 3.0
    assert upper["velocity_max_mps"] == 1.8
    assert upper["velocity_min_mps"] == 0.9
    assert lower["reverse_max_attempts"] == 500
    assert lower["wrong_way_reverse_max_attempts"] == 4


def test_static_obstacle_fast_profile_is_bounded_and_canonical():
    planner = _parameters(
        _yaml("path_following_v2/config/path_following_v2.yaml"),
        "local_trajectory_planner",
    )

    expected = {
        "max_candidates_per_side": 3,
        "static_obstacle_fast_mode": True,
        "static_analytic_candidates_per_side": 2,
        "static_analytic_extra_clearance_m": 0.04,
        "static_first_valid_side": True,
        "planning_scan_pool_size": 2,
        "lattice_fallback_to_legacy_planner": False,
        "lattice_station_step_m": 0.30,
        "lattice_lateral_step_m": 0.075,
        "lattice_beam_width": 40,
        "lattice_max_final_candidates": 2,
        "lattice_max_compute_time_ms": 6.0,
        "corridor_smoothing_iterations": 4,
        "publish_markers": False,
    }
    assert {key: planner[key] for key in expected} == expected


def test_simulator_overlays_only_contain_runtime_or_capability_keys():
    expected = {
        "path_following_v2/config/path_following_v2_sim.yaml": {
            "path_generator.use_sim_time",
            "path_generator.robot_frame",
            "path_generator.tf_timeout_sec",
            "local_trajectory_planner.use_sim_time",
            "local_trajectory_planner.robot_frame",
            "local_trajectory_planner.transform_timeout_sec",
            "path_following_v2.use_sim_time",
            "path_following_v2.robot_frame",
            "path_following_v2.marker_frame",
        },
        "reactive_control_v2/config/reactive_control_v2_sim.yaml": {
            "upper_corridor_follower.use_sim_time",
            "upper_corridor_follower.odom_topic",
            "upper_corridor_follower.base_frame",
            "lower_safety_controller.use_sim_time",
            "lower_safety_controller.odom_topic",
        },
        "drive_arbitration_v2/config/drive_arbitration_v2_sim.yaml": {
            "raceline_guard.use_sim_time",
            "drive_arbitrator.use_sim_time",
            "drive_arbitrator.require_pf_health",
        },
        "oudtra_driver_bringup/config/full_stack_sim.yaml": {
            "lower_safety_controller.use_sim_time",
            "lower_safety_controller.enable_wrong_way_recovery",
            "lower_safety_controller.enable_sim_reverse_swept_gate",
        },
    }
    for relative_path, allowed_keys in expected.items():
        assert _overlay_keys(_yaml(relative_path)) == allowed_keys


def test_simulation_inherits_production_behavior_and_does_not_enable_rl():
    path = _yaml("path_following_v2/config/path_following_v2.yaml")
    follower = _parameters(path, "path_following_v2")
    assert follower["speed_policy_mode"] == 0

    forbidden_tuning_fragments = (
        "speed_mps",
        "speed_cap",
        "planning_distance",
        "curvature",
        "lookahead",
        "lattice_",
        "clearance",
        "lateral_shift",
    )
    for relative_path in (
        "path_following_v2/config/path_following_v2_sim.yaml",
        "reactive_control_v2/config/reactive_control_v2_sim.yaml",
        "drive_arbitration_v2/config/drive_arbitration_v2_sim.yaml",
    ):
        for key in _overlay_keys(_yaml(relative_path)):
            assert not any(fragment in key for fragment in forbidden_tuning_fragments)

    master_source = (
        REPOSITORY / "oudtra_driver_bringup/launch/full_stack_launch.py"
    ).read_text(encoding="utf-8")
    assert "rl_speed_inference" not in master_source
    assert "ppo_speed" not in master_source


def test_runtime_uses_the_published_raceline_as_its_only_frenet_reference():
    path = _yaml("path_following_v2/config/path_following_v2.yaml")
    generator = _parameters(path, "path_generator")
    planner = _parameters(path, "local_trajectory_planner")
    publisher = _parameters(
        _yaml("centerline_tools/config/raceline_publisher.yaml"),
        "raceline_publisher",
    )

    assert planner["raceline_reference_topic"] == publisher["path_topic"]
    assert generator["raceline_waypoints_topic"] == publisher["waypoints_topic"]
    assert planner["require_raceline_reference"] is True
    assert planner["raceline_reference_match_tolerance_m"] > 0.0
    removed_parameters = {
        "centerline_csv_path",
        "centerline_direction",
        "centerline_frame",
        "centerline_closed_loop",
        "require_centerline_reference",
    }
    assert removed_parameters.isdisjoint(planner)

    launch_files = (
        "path_following_v2/launch/path_following_v2_launch.py",
        "path_following_v2/launch/path_following_v2_sim_launch.py",
        "oudtra_driver_bringup/launch/full_stack_launch.py",
        "oudtra_driver_bringup/launch/full_stack_onboard_launch.py",
        "oudtra_driver_bringup/launch/full_stack_sim_launch.py",
    )
    for relative_path in launch_files:
        source = (REPOSITORY / relative_path).read_text(encoding="utf-8")
        assert "centerline_csv_path" not in source
        assert "centerline_direction" not in source


def test_onboard_safety_is_not_relaxed_by_the_common_integration_profile():
    reactive = _yaml("reactive_control_v2/config/reactive_control_v2.yaml")
    lower = _parameters(reactive, "lower_safety_controller")
    integration = _parameters(
        _yaml("oudtra_driver_bringup/config/full_stack.yaml"),
        "lower_safety_controller",
    )
    arbitration = _parameters(
        _yaml("drive_arbitration_v2/config/drive_arbitration_v2.yaml"),
        "drive_arbitrator",
    )

    assert lower["enable_wrong_way_recovery"] is False
    assert lower["enable_sim_reverse_swept_gate"] is False
    assert lower["enable_raceline_stall_handoff"] is False
    assert "enable_wrong_way_recovery" not in integration
    assert "enable_sim_reverse_swept_gate" not in integration
    assert arbitration["require_pf_health"] is True


def test_master_and_compatibility_launches_only_compose_child_launches():
    launch_directory = REPOSITORY / "oudtra_driver_bringup/launch"
    for filename in (
        "full_stack_launch.py",
        "full_stack_sim_launch.py",
        "full_stack_onboard_launch.py",
    ):
        tree = ast.parse((launch_directory / filename).read_text(encoding="utf-8"))
        node_calls = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "Node"
        ]
        assert not node_calls

    shared_source = (launch_directory / "full_stack_launch.py").read_text(
        encoding="utf-8"
    )
    for child_launch in (
        "raceline_publisher.launch.py",
        "path_following_v2_launch.py",
        "drive_arbitration_v2.launch.py",
        "reactive_control_v2_launch.py",
    ):
        assert child_launch in shared_source
    assert "scoped=True" in shared_source
