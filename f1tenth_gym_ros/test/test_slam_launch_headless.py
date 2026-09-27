"""The localization bridge keeps RViz optional without changing manual defaults."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'slam_launch', ROOT / 'launch/gym_bridge_slam_launch.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_manual_rviz_default_and_headless_condition():
    description = MODULE.generate_launch_description()
    arguments = {action.name: action for action in description.entities
                 if isinstance(action, DeclareLaunchArgument)}
    assert arguments['use_rviz'].default_value[0].text == 'true'
    context = LaunchContext()
    context.launch_configurations['config_file'] = str(
        ROOT / 'config/sim_ifac_roboracer.yaml')
    context.launch_configurations['slam_mapping_on'] = 'false'
    context.launch_configurations['use_rviz'] = 'false'
    nodes = MODULE.launch_nodes(context)
    rviz = next(node for node in nodes if node.condition is not None)
    assert rviz.condition.evaluate(context) is False
    context.launch_configurations['use_rviz'] = 'true'
    assert rviz.condition.evaluate(context) is True
