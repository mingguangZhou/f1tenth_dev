"""Persistent PF-compatible PnC; bringup/simulator and PF stay separate."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(__file__).with_name('full_stack_localization_launch.py'))),
        launch_arguments={'platform': 'onboard'}.items())])
