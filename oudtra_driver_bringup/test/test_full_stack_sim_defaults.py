"""Contracts for fixed simulator and onboard racing-line locations."""

import ast
import hashlib
from pathlib import Path

import yaml


REPOSITORY = Path(__file__).resolve().parents[2]
LAUNCH_DIRECTORY = REPOSITORY / "oudtra_driver_bringup/launch"
IFAC_DIRECTORY = Path("centerline_tools/output_backup/ifac_roboracer")
RACELINE_RELATIVE = IFAC_DIRECTORY / "raceline_points_optimized.csv"
CENTERLINE_RELATIVE = IFAC_DIRECTORY / "centerline_points_smooth.csv"
REPORT_RELATIVE = IFAC_DIRECTORY / "raceline_points_optimized_validation.yaml"


def _launch_argument_nodes(filename):
    tree = ast.parse((LAUNCH_DIRECTORY / filename).read_text(encoding="utf-8"))
    defaults = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Dict):
            continue
        for key, value in zip(node.keys, node.values):
            if not isinstance(key, ast.Constant) or not isinstance(key.value, str):
                continue
            defaults[key.value] = value
    return defaults


def _literal_default(nodes, name):
    return ast.literal_eval(nodes[name])


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_fixed_raceline_retains_offline_validation_provenance():
    raceline = REPOSITORY / RACELINE_RELATIVE
    centerline = REPOSITORY / CENTERLINE_RELATIVE
    report = yaml.safe_load(
        (REPOSITORY / REPORT_RELATIVE).read_text(encoding="utf-8")
    )

    assert raceline.is_file()
    assert centerline.is_file()
    assert report["outputs"]["raceline"] == RACELINE_RELATIVE.as_posix()
    assert report["outputs"]["raceline_sha256"] == _sha256(raceline)
    assert report["inputs"]["centerline_sha256"] == _sha256(centerline)
    assert report["inputs"]["baseline_raceline"] is None
    assert all(
        value
        for value in report["validation"].values()
        if isinstance(value, bool)
    )


def test_wrappers_use_the_fixed_platform_raceline_locations():
    onboard = _launch_argument_nodes("full_stack_onboard_launch.py")
    simulator = _launch_argument_nodes("full_stack_sim_launch.py")

    assert _literal_default(onboard, "raceline_csv_path") == (
        "/f1tenth_ws/src/f1tenth_dev/" + RACELINE_RELATIVE.as_posix()
    )
    assert _literal_default(simulator, "raceline_csv_path") == (
        "/sim_ws/src/" + RACELINE_RELATIVE.as_posix()
    )
    assert _literal_default(onboard, "raceline_direction") == "csv"
    assert _literal_default(simulator, "raceline_direction") == "csv"
    assert (REPOSITORY / RACELINE_RELATIVE).is_file()

    assert onboard.keys() == simulator.keys()
    for name in onboard.keys() - {"raceline_csv_path"}:
        assert ast.dump(onboard[name]) == ast.dump(simulator[name])
    assert "track" not in onboard
    assert "centerline_csv_path" not in onboard
    assert "centerline_direction" not in onboard

    onboard_source = (
        LAUNCH_DIRECTORY / "full_stack_onboard_launch.py"
    ).read_text(encoding="utf-8")
    simulator_source = (
        LAUNCH_DIRECTORY / "full_stack_sim_launch.py"
    ).read_text(encoding="utf-8")
    assert 'forwarded["platform"] = "onboard"' in onboard_source
    assert 'forwarded["platform"] = "sim"' in simulator_source


def test_shared_launch_requires_a_supplied_raceline_without_workspace_assumptions():
    source = (LAUNCH_DIRECTORY / "full_stack_launch.py").read_text(
        encoding="utf-8"
    )
    assert "/sim_ws/" not in source
    assert "/f1tenth_ws/" not in source
    assert "raceline_csv_path must not be empty" in source
    assert "centerline_csv_path" not in source
    assert "centerline_direction" not in source
    assert 'DeclareLaunchArgument("track"' not in source
    assert "track.yaml" not in source
    assert "_resolve_track" not in source
    assert "sha256" not in source


def test_centerline_tools_remains_only_the_preinstalled_runtime_publisher():
    source = (LAUNCH_DIRECTORY / "full_stack_launch.py").read_text(
        encoding="utf-8"
    )
    assert "raceline_publisher.launch.py" in source
    setup_source = (REPOSITORY / "centerline_tools/setup.py").read_text(
        encoding="utf-8"
    )
    assert "track_data_files" not in setup_source
