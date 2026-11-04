"""The manual PnC adapter hides plumbing without changing effective settings."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture
def adapter(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'manual_pnc_test', PACKAGE / 'launch/full_stack_localization_launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    map_yaml = tmp_path / 'track.yaml'
    map_yaml.write_text('image: track.pgm\n')
    context = LaunchContext()
    context.launch_configurations.update(
        platform='sim', map_yaml=str(map_yaml), selected_raceline=str(tmp_path / 'selected_raceline.yaml'))
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    settings = tmp_path / 'settings'
    settings.mkdir()
    monkeypatch.setattr(module.tempfile, 'mkdtemp', lambda prefix: str(settings))
    return module, context, settings


def test_manual_pnc_preserves_timed_settings_and_starts_no_pf(adapter, monkeypatch):
    module, context, settings = adapter
    selection = dict(raceline_csv='/explicit/reference.csv', direction='reverse',
                     candidate_id='008_geometry', status='experimental',
                     manifest=context.launch_configurations['selected_raceline'],
                     map_yaml=context.launch_configurations['map_yaml'])

    def resolve(manifest, allow, map_yaml):
        assert manifest == selection['manifest']
        assert allow is True
        assert map_yaml == context.launch_configurations['map_yaml']
        return selection

    monkeypatch.setattr(module, 'resolve_selection', resolve)
    context.launch_configurations['allow_experimental'] = 'true'
    actions = module._launch_pnc(context)
    includes = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    assert len(includes) == 1
    args = dict(includes[0].launch_arguments)
    assert args['raceline_csv_path'] == selection['raceline_csv']
    assert args['raceline_direction'] == 'reverse'
    assert args['use_sim_time'] == args['start_particle_filter'] == 'false'
    overlay = settings / 'localization_eval_with_speed_limits.yaml'
    for key in ('path_platform_config', 'reactive_platform_config',
                'arbitration_platform_config', 'integration_platform_config'):
        assert args[key] == str(overlay)

    spec = importlib.util.spec_from_file_location(
        'merge_contract', PACKAGE / 'scripts/prepare_localization_overlay.py')
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    merged = yaml.safe_load(overlay.read_text())
    assert merged == helper.speed_profile_overlay(
        PACKAGE / 'config/localization_eval_sim.yaml',
        PACKAGE / 'config/comparison_speed_limits.yaml')
    safety = merged['lower_safety_controller']['ros__parameters']
    assert safety['enable_sim_reverse_swept_gate'] is False
    assert safety['odom_topic'] == '/pf/pose/odom'
    assert safety['absolute_speed_limit_mps'] == 2.0
    assert merged['drive_arbitrator']['ros__parameters']['require_pf_health'] is True
    assert overlay.with_suffix('.provenance.yaml').is_file()


def test_experimental_selection_requires_explicit_acknowledgment(adapter, monkeypatch):
    module, context, settings = adapter

    def resolve(manifest, allow, map_yaml):
        if not allow:
            raise ValueError('Experimental selection requires --allow-experimental')

    monkeypatch.setattr(module, 'resolve_selection', resolve)
    with pytest.raises(RuntimeError, match='requires allow_experimental:=true'):
        module._launch_pnc(context)
    assert not list(settings.iterdir())


@pytest.mark.parametrize('key,value,error', [
    ('map_yaml', '/missing/map.yaml', 'existing absolute map'),
    ('map_yaml', 'relative.yaml', 'existing absolute map'),
    ('selected_raceline', '', 'Provide track or selected_raceline'),
    ('allow_experimental', 'yes', 'true or false'),
])
def test_invalid_inputs_fail_before_launch(adapter, key, value, error):
    module, context, settings = adapter
    context.launch_configurations[key] = value
    with pytest.raises(RuntimeError, match=error):
        module._launch_pnc(context)
    assert not list(settings.iterdir())


def test_bad_speed_profile_leaves_no_temporary_settings(adapter, monkeypatch, tmp_path):
    module, context, settings = adapter
    monkeypatch.setattr(module, 'resolve_selection', lambda *args: {'map_yaml': context.launch_configurations['map_yaml']})
    bad_profile = tmp_path / 'bad.yaml'
    bad_profile.write_text('{}')
    context.launch_configurations['speed_profile'] = str(bad_profile)
    with pytest.raises(ValueError, match='Speed profile requires'):
        module._launch_pnc(context)
    assert not settings.exists()


def test_track_and_manifest_cannot_both_be_selected(adapter):
    module, context, _ = adapter
    context.launch_configurations['track'] = 'tempo_coffee'
    with pytest.raises(RuntimeError, match='Choose track OR'):
        module._launch_pnc(context)


def test_onboard_adapter_keeps_platform_defaults_and_only_overlays_speeds(adapter, monkeypatch):
    module, context, _ = adapter
    context.launch_configurations.update(platform='onboard', track='tempo_coffee',
                                        map_yaml='', selected_raceline='', allow_experimental='true')
    def resolve(track, allow, map_yaml):
        assert (track, allow, map_yaml) == ('tempo_coffee', True, None)
        return dict(raceline_csv='/portable/reference.csv', direction='csv',
                    candidate_id='008_geometry', status='experimental',
                    map_yaml='/portable/map.yaml', manifest='/portable/selected_raceline.yaml')
    monkeypatch.setattr(module, 'resolve_track', resolve)
    actions = module._launch_pnc(context)
    include = next(a for a in actions if isinstance(a, IncludeLaunchDescription))
    args = dict(include.launch_arguments)
    assert args['start_particle_filter'] == args['use_sim_time'] == 'false'
    for key in ('path_platform_config', 'reactive_platform_config',
                'arbitration_platform_config', 'integration_platform_config'):
        assert args[key] == context.launch_configurations['speed_profile']
        data = yaml.safe_load(Path(args[key]).read_text())
        assert set(data) == {'/**', 'lower_safety_controller'}
        assert set(data['lower_safety_controller']['ros__parameters']) == {'absolute_speed_limit_mps'}
    assert args['raceline_csv_path'] == '/portable/reference.csv'
