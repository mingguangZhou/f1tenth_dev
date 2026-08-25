"""Contracts for simulator full-stack artifact selection."""

import ast
import hashlib
from pathlib import Path

import yaml


REPOSITORY = Path(__file__).resolve().parents[2]
LAUNCH_FILE = (
    REPOSITORY / "oudtra_driver_bringup/launch/full_stack_sim_launch.py"
)
IFAC_DIRECTORY = Path("centerline_tools/output_backup/ifac_roboracer")
OPTIMIZED_RELATIVE = IFAC_DIRECTORY / "raceline_points_optimized.csv"
CENTERLINE_RELATIVE = IFAC_DIRECTORY / "centerline_points_smooth.csv"
REPORT_RELATIVE = IFAC_DIRECTORY / "raceline_points_optimized_validation.yaml"


def _launch_arguments():
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    defaults = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name):
            continue
        if node.func.id != "DeclareLaunchArgument" or not node.args:
            continue
        name = ast.literal_eval(node.args[0])
        for keyword in node.keywords:
            if keyword.arg == "default_value":
                try:
                    defaults[name] = ast.literal_eval(keyword.value)
                except (TypeError, ValueError):
                    defaults[name] = None
                break
    return defaults


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_simulator_defaults_to_validated_optimized_raceline():
    defaults = _launch_arguments()
    expected_runtime_path = f"/sim_ws/src/{OPTIMIZED_RELATIVE.as_posix()}"
    assert defaults["raceline_csv_path"] == expected_runtime_path
    assert "centerline_csv_path" not in defaults
    assert "centerline_direction" not in defaults

    optimized = REPOSITORY / OPTIMIZED_RELATIVE
    centerline = REPOSITORY / CENTERLINE_RELATIVE
    report = yaml.safe_load(
        (REPOSITORY / REPORT_RELATIVE).read_text(encoding="utf-8")
    )

    assert optimized.is_file()
    assert centerline.is_file()
    # The centerline remains valid offline optimizer provenance, but it is no
    # longer a runtime planning input or launch argument.
    assert optimized.read_bytes() != centerline.read_bytes()
    assert report["outputs"]["raceline"] == OPTIMIZED_RELATIVE.as_posix()
    assert report["outputs"]["raceline_sha256"] == _sha256(optimized)
    assert report["inputs"]["centerline_sha256"] == _sha256(centerline)
    assert report["inputs"]["baseline_raceline"] is None
    assert all(
        value
        for value in report["validation"].values()
        if isinstance(value, bool)
    )
