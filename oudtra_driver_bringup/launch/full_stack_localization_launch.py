"""Shared explicit-selection adapter; preserve each platform's control defaults."""

import importlib.util
from pathlib import Path
import shutil
import tempfile

import yaml

from ament_index_python.packages import (
    get_package_prefix, get_package_share_directory,
)
from centerline_tools.raceline_selection import resolve_selection, resolve_track
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_pnc(context):
    share = Path(get_package_share_directory('oudtra_driver_bringup'))
    platform = LaunchConfiguration('platform').perform(context)
    if platform not in ('sim', 'onboard'):
        raise RuntimeError('platform must be sim or onboard')
    track = LaunchConfiguration('track').perform(context).strip()
    map_yaml = LaunchConfiguration('map_yaml').perform(context).strip()
    manifest = LaunchConfiguration('selected_raceline').perform(context).strip()
    acknowledgment = LaunchConfiguration('allow_experimental').perform(context).lower()
    if acknowledgment not in ('true', 'false'):
        raise RuntimeError('allow_experimental must be true or false')
    if map_yaml and (not Path(map_yaml).is_absolute() or not Path(map_yaml).is_file()):
        raise RuntimeError('map_yaml must be an existing absolute map YAML path')
    if track and manifest:
        raise RuntimeError('Choose track OR selected_raceline, not both')
    if not track and not manifest:
        raise RuntimeError('Provide track or selected_raceline with explicit map_yaml')
    if not track and not map_yaml:
        raise RuntimeError('map_yaml is required with selected_raceline')
    try:
        selection = (resolve_track(track, acknowledgment == 'true', map_yaml or None) if track
                     else resolve_selection(manifest, acknowledgment == 'true', map_yaml))
        map_yaml = selection['map_yaml']
    except ValueError as error:
        # The shared resolver also serves a CLI; show this launch's syntax here.
        raise RuntimeError(str(error).replace(
            '--allow-experimental', 'allow_experimental:=true')) from error

    # Reuse the timed runner's merge and provenance writer, installed beside it.
    helper_path = (Path(get_package_prefix('oudtra_driver_bringup')) / 'lib' /
                   'oudtra_driver_bringup' / 'prepare_localization_overlay.py')
    spec = importlib.util.spec_from_file_location('manual_sim_overlay', helper_path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    speed_profile = LaunchConfiguration('speed_profile').perform(context)
    cleanup_actions = []
    if platform == 'sim':
        temporary = Path(tempfile.mkdtemp(prefix='roboracer_manual_pnc_'))
        overlay = temporary / 'localization_eval_with_speed_limits.yaml'
        try:
            helper.write_overlay(share / 'config' / 'localization_eval_sim.yaml', speed_profile, overlay)
            effective = yaml.safe_load(overlay.read_text())
        except Exception:
            shutil.rmtree(temporary)
            raise

        def cleanup(_context):
            shutil.rmtree(temporary, ignore_errors=True)

        cleanup_actions = [RegisterEventHandler(OnShutdown(
            on_shutdown=[OpaqueFunction(function=cleanup)]))]
        config_description = 'Internal temporary settings (removed on shutdown): ' + str(overlay)
    else:
        # Onboard defaults already own its frames/time/health/safety. Only speeds
        # are overlaid; simulator integration settings never enter this branch.
        effective = helper.load_speed_profile(speed_profile)
        overlay = Path(speed_profile)
        config_description = 'Onboard defaults plus speed-only profile: ' + str(overlay)

    arguments = {
        'use_sim_time': 'false',
        'start_particle_filter': 'false',
        'raceline_csv_path': selection['raceline_csv'],
        'raceline_direction': selection['direction'],
        'path_platform_config': str(overlay),
        'reactive_platform_config': str(overlay),
        'arbitration_platform_config': str(overlay),
        'integration_platform_config': str(overlay),
    }
    return cleanup_actions + [
        LogInfo(msg='MANUAL PF ' + platform.upper() + ' — PnC only; no automatic recording'),
        LogInfo(msg='Selected map: ' + str(Path(map_yaml).resolve())),
        LogInfo(msg='Raceline: {} | {} | {} | direction {}'.format(
            selection['candidate_id'], selection['status'].upper(),
            selection['raceline_csv'], selection['direction'])),
        LogInfo(msg='Selection manifest: ' + selection['manifest']),
        LogInfo(msg='Speed profile: ' + LaunchConfiguration('speed_profile').perform(context)),
        LogInfo(msg='Forward command range: {}–{} m/s; absolute command cap: {} m/s'.format(
            effective['/**']['ros__parameters']['command_speed_min_mps'],
            effective['/**']['ros__parameters']['command_speed_max_mps'],
            effective['lower_safety_controller']['ros__parameters']['absolute_speed_limit_mps'])),
        LogInfo(msg='PF health required; platform safety retained; wall time.'),
        LogInfo(msg=config_description),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / ('full_stack_' + platform + '_launch.py'))),
            launch_arguments=arguments.items()),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('oudtra_driver_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('platform', description='sim or onboard; selected by the entry point'),
        DeclareLaunchArgument('track', default_value='', description='Named curated selection, e.g. tempo_coffee'),
        DeclareLaunchArgument('map_yaml', default_value='', description='Explicit PF map; checked against selection'),
        DeclareLaunchArgument('selected_raceline', default_value='', description='Explicit selected_raceline.yaml'),
        DeclareLaunchArgument('allow_experimental', default_value='false',
                              description='Acknowledge failed production validation for this experiment'),
        DeclareLaunchArgument('speed_profile', default_value=str(
            share / 'config' / 'comparison_speed_limits.yaml')),
        OpaqueFunction(function=_launch_pnc),
    ])
