"""Race-day selection and UX contracts, without vehicle nodes or GUI clicks."""
import importlib.util
from pathlib import Path

import pytest
import yaml

from test_raceline_workbench import ROOT, circle, fixture_bench, fake_solver, write_phase9, wb, opt
from centerline_tools.raceline_selection import resolve_selection, handoff_arguments
from centerline_tools.raceline_workbench_view import action_states, preset_changes, create_workbench_figure


def failed_candidate(bench, monkeypatch):
    def failed(*args, **kwargs):
        result = fake_solver(*args, **kwargs)
        result.points = circle(0.8)
        return result
    monkeypatch.setattr(opt, 'optimize_global_raceline', failed)
    return bench.run_baseline()


def test_gui_default_and_headless_override(monkeypatch):
    monkeypatch.setattr(wb, 'interactive_display_available', lambda: True)
    assert wb.use_gui()
    assert not wb.use_gui(headless=True)
    monkeypatch.setattr(wb, 'interactive_display_available', lambda: False)
    assert not wb.use_gui()
    with pytest.raises(RuntimeError, match='No working display'):
        wb.use_gui(explicit_gui=True)


def test_fresh_session_names_are_unique_and_track_explicit(tmp_path):
    bench = fixture_bench(tmp_path)
    first = wb.default_output('race_day_track.pgm', bench.identity)
    second = wb.default_output('race_day_track.pgm', bench.identity)
    assert first != second and first.name.startswith('race_day_track-')
    assert first.parent.name == 'workbench'


def test_presets_map_to_search_only_and_reset_baseline(tmp_path):
    bench = fixture_bench(tmp_path)
    choices = {group: 'Balanced' for group in bench.settings['tuning_presets']}
    changes = preset_changes(bench, choices)
    assert wb.search_profile(bench.baseline, changes, bench.settings['search']['ranges']) == bench.baseline
    choices.update({'Corner Cutting': 'More', 'Steering Smoothness': 'More', 'Optimization Detail': 'Fine'})
    config = wb.search_profile(bench.baseline, preset_changes(bench, choices), bench.settings['search']['ranges'])
    assert config.path_length_weight == 0.25
    assert config.curvature_rate_weight == 0.08 and config.offset_slope_weight == 0.03
    assert config.optimization_spacing_m == 0.05
    assert all(getattr(config, key) == getattr(bench.baseline, key) for key in wb.HARD_KEYS)


def test_gui_steps_busy_guard_and_advanced_controls(tmp_path, monkeypatch):
    import matplotlib.pyplot as plt
    bench = fixture_bench(tmp_path)
    assert action_states(bench)['Generate Raceline']
    assert not action_states(bench)['Select / Export Validated']
    failed_candidate(bench, monkeypatch)
    assert action_states(bench)['Export Experimental']
    assert not any(action_states(bench, busy=True).values())
    fig = create_workbench_figure(bench)
    controls = fig.workbench_controls
    assert len(controls['radios']) == 3
    assert not controls['buttons']['Select / Export Validated'].active
    calls = []
    inner = controls['action'](lambda: calls.append('unexpected concurrent operation'))
    controls['action'](lambda: (calls.append('one'), inner()))()
    assert calls == ['one']
    controls['advanced']()
    popup = controls['popups'][-1]
    assert set(popup.workbench_controls['sliders']) == set(wb.SEARCH_KEYS)
    popup.workbench_controls['sliders']['curvature_rate_weight'].set_val(0.04)
    assert controls['tuning']['curvature_rate_weight'] == bench.baseline.curvature_rate_weight
    popup.workbench_controls['apply']._observers.process('clicked', None)
    controls['radios']['Corner Cutting'].set_active(2)
    assert controls['tuning']['path_length_weight'] == 0.25
    assert controls['tuning']['curvature_rate_weight'] == 0.04  # other control's advanced edit survives
    plt.close('all')


def test_gui_experimental_confirmation_is_explicit(tmp_path, monkeypatch):
    import matplotlib.pyplot as plt
    bench = fixture_bench(tmp_path)
    failed_candidate(bench, monkeypatch)
    fig = create_workbench_figure(bench)
    fig.workbench_controls['confirm_experimental']()
    dialog = fig.workbench_controls['popups'][-1]
    assert not (bench.output / 'selected_raceline.yaml').exists()
    assert any('NOT validated' in t.get_text() for t in dialog.texts)
    dialog.workbench_controls['cancel']._observers.process('clicked', None)
    assert not (bench.output / 'selected_raceline.yaml').exists()
    fig.workbench_controls['confirm_experimental']()
    dialog = fig.workbench_controls['popups'][-1]
    dialog.workbench_controls['confirm']._observers.process('clicked', None)
    assert resolve_selection(bench.output / 'selected_raceline.yaml', True)['status'] == 'experimental'
    plt.close('all')


def test_validated_selection_resolves_existing_interfaces(tmp_path, monkeypatch):
    bench = fixture_bench(tmp_path)
    monkeypatch.setattr(opt, 'optimize_global_raceline', fake_solver)
    bench.run_baseline()
    assert not (bench.output / 'selected_raceline.yaml').exists()  # generation is not selection
    selected = bench.select_export()
    resolved = resolve_selection(bench.output / 'selected_raceline.yaml', map_yaml=bench.identity['map_yaml'])
    assert resolved == selected and selected['status'] == 'validated'
    assert handoff_arguments(selected, 'runner') == ['--raceline-csv', selected['raceline_csv'], '--raceline-direction', 'csv']
    assert handoff_arguments(selected, 'pnc') == ['raceline_csv_path:=' + selected['raceline_csv'], 'raceline_direction:=csv']
    assert handoff_arguments(selected, 'publisher') == ['csv_path:=' + selected['raceline_csv'], 'direction:=csv']
    before = (bench.output / 'selected_raceline.yaml').read_bytes()
    bench.select_export()
    assert (bench.output / 'selected_raceline.yaml').read_bytes() == before


def test_experimental_exact_geometry_and_not_production(tmp_path, monkeypatch):
    bench = fixture_bench(tmp_path)
    c = failed_candidate(bench, monkeypatch)
    with pytest.raises(ValueError, match='confirmation'):
        bench.select_export(experimental=True)
    with pytest.raises(ValueError, match='promotion is disabled'):
        bench.select_export()
    s = bench.select_export(experimental=True, confirmed=True)
    assert Path(s['raceline_csv']).read_bytes() == Path(c['path']).read_bytes()
    assert not (bench.output / 'raceline_points_optimized.csv').exists()
    evidence = yaml.safe_load(Path(s['validation_report']).read_text())
    assert not evidence['valid'] and evidence['failure_reasons'] and evidence['status'] == 'experimental'
    with pytest.raises(ValueError, match='allow-experimental'):
        resolve_selection(s['manifest'])
    assert resolve_selection(s['manifest'], True)['status'] == 'experimental'


def test_reference_preview_cannot_be_exported(tmp_path, monkeypatch):
    bench = fixture_bench(tmp_path)
    def early_failure(*args, **kwargs):
        raise ValueError('no legal lateral bounds')
    monkeypatch.setattr(opt, 'optimize_global_raceline', early_failure)
    c = bench.run_baseline()
    assert c['candidate_kind'] == 'reference_preview' and c['path'] is None
    with pytest.raises(ValueError, match='Reference-only'):
        bench.select_export(experimental=True, confirmed=True)


@pytest.mark.parametrize('asset', ['raceline_csv', 'map_yaml', 'map_image', 'validation_report'])
@pytest.mark.parametrize('change', ['missing', 'changed'])
def test_missing_or_changed_selection_assets_fail(tmp_path, monkeypatch, asset, change):
    bench = fixture_bench(tmp_path)
    monkeypatch.setattr(opt, 'optimize_global_raceline', fake_solver)
    bench.run_baseline()
    s = bench.select_export()
    path = Path(s[asset])
    if change == 'missing':
        path.rename(path.with_suffix(path.suffix + '.removed'))
    else:
        path.write_bytes(path.read_bytes() + b'changed')
    with pytest.raises(ValueError, match='missing|hash mismatch'):
        resolve_selection(s['manifest'])


def test_manifest_fields_and_caller_map_are_authoritative(tmp_path, monkeypatch):
    bench = fixture_bench(tmp_path)
    monkeypatch.setattr(opt, 'optimize_global_raceline', fake_solver)
    bench.run_baseline()
    s = bench.select_export()
    wrong = tmp_path / 'wrong.yaml'
    wrong.write_text('image: different.png\n')
    with pytest.raises(ValueError, match='differs'):
        resolve_selection(s['manifest'], map_yaml=wrong)
    data = yaml.safe_load(Path(s['manifest']).read_text())
    data['selection']['vehicle_profile_sha256'] = 'wrong'
    Path(s['manifest']).write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='disagree'):
        resolve_selection(s['manifest'])


def test_reopen_preserves_candidates_selection_and_stale_guard(tmp_path, monkeypatch):
    bench = fixture_bench(tmp_path)
    monkeypatch.setattr(opt, 'optimize_global_raceline', fake_solver)
    bench.run_baseline()
    s = bench.select_export()
    reopened = wb.Workbench.reopen(bench.output)
    assert reopened.candidates == bench.candidates and reopened.active_selection == s
    reopened.run_baseline()  # another trial must not mutate active assets/manifest
    assert resolve_selection(s['manifest']) == s
    with pytest.raises(ValueError, match='active selection'):
        reopened.promote()
    Path(bench.identity['map_yaml']).write_text('image: changed.png\n')
    with pytest.raises((ValueError, OSError)):
        wb.Workbench.reopen(bench.output)


def test_manual_direction_not_double_reversed(tmp_path):
    bench = fixture_bench(tmp_path)
    manual = tmp_path / 'manual.csv'
    opt.save_path_csv(bench.centerline[::-1], manual)
    bench.import_manual(manual, 'reverse')
    s = bench.select_export()
    assert s['generator'] == 'manual_editor' and s['candidate_direction'] == 'reverse' and s['direction'] == 'csv'


def test_runner_boundary_gates_and_provenance(tmp_path, monkeypatch):
    bench = fixture_bench(tmp_path)
    failed_candidate(bench, monkeypatch)
    s = bench.select_export(experimental=True, confirmed=True)
    path = ROOT.parent / 'oudtra_driver_bringup/scripts/run_localization_smoke.py'
    spec = importlib.util.spec_from_file_location('smoke_selection_test', path)
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    with pytest.raises(ValueError, match='allow-experimental'):
        runner.resolve_raceline_input(s['manifest'], None, None, bench.identity['map_yaml'])
    csv_path, direction, metadata = runner.resolve_raceline_input(s['manifest'], None, None, bench.identity['map_yaml'], True)
    assert csv_path == s['raceline_csv'] and direction == 'csv' and metadata == s
    assert metadata['manifest_sha256'] == opt.file_sha256(s['manifest'])
    from types import SimpleNamespace
    record_root = tmp_path / 'recorded'
    record_root.mkdir()
    recording = SimpleNamespace(root=record_root, metadata={}, save=lambda: None)
    runner.preserve_raceline_selection(recording, metadata)
    assert recording.metadata['selected_raceline'] == s
    assert (record_root / 'selected_raceline.yaml').read_bytes() == Path(s['manifest']).read_bytes()
    assert (record_root / 'selected_raceline_validation.yaml').read_bytes() == Path(s['validation_report']).read_bytes()
    assert runner.pnc_launch_arguments(csv_path, direction, '/tmp/overlay')[1:3] == handoff_arguments(s, 'pnc')
    with pytest.raises(ValueError, match='disagrees'):
        runner.resolve_raceline_input(s['manifest'], None, 'reverse', bench.identity['map_yaml'], True)
    assert runner.resolve_raceline_input(None, '/tmp/explicit.csv', 'reverse', '/tmp/map') == ('/tmp/explicit.csv', 'reverse', None)
    with pytest.raises(ValueError):
        runner.resolve_raceline_input(None, '/tmp/explicit.csv', None, '/tmp/map')


def test_wheel_zoom_and_refresh_preserve_view(tmp_path):
    from types import SimpleNamespace
    from centerline_tools.raceline_workbench_view import zoom_at_cursor
    import matplotlib.pyplot as plt
    fig = create_workbench_figure(fixture_bench(tmp_path))
    ax = fig.workbench_controls['map_axis']
    before = ax.get_xlim()
    zoom_at_cursor(ax, SimpleNamespace(inaxes=ax, xdata=0.0, ydata=0.0, button='up'))
    zoomed = ax.get_xlim()
    assert zoomed[1] - zoomed[0] == pytest.approx((before[1] - before[0]) * 0.8)
    fig.workbench_controls['refresh']()
    assert ax.get_xlim() == pytest.approx(zoomed)
    zoom_at_cursor(ax, SimpleNamespace(inaxes=ax, xdata=0.0, ydata=0.0, button='down'))
    assert ax.get_xlim() == pytest.approx(before)
    plt.close('all')


def test_preset_popup_resolves_session_specific_balanced_values(tmp_path):
    import matplotlib.pyplot as plt
    bench = fixture_bench(tmp_path)
    fig = create_workbench_figure(bench)
    fig.workbench_controls['preset_values']()
    rows = fig.workbench_controls['popups'][-1].workbench_controls['rows']
    assert len(rows) == 9
    assert any(row[1] == 'Fine' and 'optimization_spacing_m = 0.05' in row[2] for row in rows)
    assert any(row[0] == 'Steering Smoothness' and row[1] == 'Balanced' and
               f'curvature_rate_weight = {bench.baseline.curvature_rate_weight:g}' in row[2] for row in rows)
    plt.close('all')


def test_advanced_typed_dragged_values_and_cancel(tmp_path):
    import matplotlib.pyplot as plt
    bench = fixture_bench(tmp_path)
    fig = create_workbench_figure(bench)
    controls = fig.workbench_controls
    controls['advanced']()
    editor = controls['popups'][-1].workbench_controls
    editor['entries']['path_length_weight'].set_val('0.18')
    assert editor['sliders']['path_length_weight'].val == 0.18
    assert editor['pending']['path_length_weight'] == 0.18
    editor['sliders']['path_length_weight'].set_val(0.20)
    assert float(editor['entries']['path_length_weight'].text) == 0.20
    for bad in ('nan', 'invalid', '999'):
        editor['entries']['path_length_weight'].set_val(bad)
        assert editor['pending']['path_length_weight'] == 0.20
    editor['entries']['safety_margin_m'].set_val('0.35')
    editor['entries']['maximum_curvature_inv_m'].set_val('0.95')
    assert editor['limits'] == {'safety_margin_m': 0.35, 'maximum_curvature_inv_m': 0.95}
    editor['cancel']._observers.process('clicked', None)
    assert controls['tuning']['path_length_weight'] == bench.baseline.path_length_weight
    assert bench.baseline.safety_margin_m == 0.25
    plt.close('all')


def test_deliberate_limit_profile_forks_without_inherited_validation(tmp_path, monkeypatch):
    bench = fixture_bench(tmp_path)
    monkeypatch.setattr(opt, 'optimize_global_raceline', fake_solver)
    bench.run_baseline()
    selected = bench.select_export()
    before = Path(selected['manifest']).read_bytes()
    monkeypatch.setattr(wb, 'default_output', lambda image, identity: tmp_path / 'new_profile')
    with pytest.raises(ValueError, match='confirmation'):
        bench.with_vehicle_limits({'safety_margin_m': 0.35})
    with pytest.raises(ValueError, match='range'):
        bench.with_vehicle_limits({'maximum_curvature_inv_m': 10}, confirmed=True)
    new = bench.with_vehicle_limits({'safety_margin_m': 0.35, 'maximum_curvature_inv_m': 0.95}, confirmed=True)
    assert new.output != bench.output and not new.candidates and not new.outputs and not new.active_selection
    assert new.baseline.safety_margin_m == 0.35 and new.baseline.maximum_curvature_inv_m == 0.95
    assert new.baseline.validation_step_m == bench.baseline.validation_step_m
    assert (new.output / 'session_profile.yaml').exists()
    assert Path(selected['manifest']).read_bytes() == before
    assert resolve_selection(selected['manifest']) == selected
    assert all(config.maximum_curvature_inv_m == 0.95 and config.safety_margin_m == 0.35
               for phase, config in wb.search_schedule(new.baseline, new.settings))
    write_phase9(new, circle())
    new.run_baseline()
    declared = new.select_export()
    assert declared['status'] == 'validated' and declared['custom_vehicle_profile']
    assert declared['vehicle_constraints']['maximum_curvature_inv_m'] == 0.95
    assert resolve_selection(declared['manifest']) == declared


def test_gui_limit_changes_require_confirmation(tmp_path, monkeypatch):
    import matplotlib.pyplot as plt
    bench = fixture_bench(tmp_path)
    monkeypatch.setattr(wb, 'default_output', lambda image, identity: tmp_path / 'gui_profile')
    fig = create_workbench_figure(bench)
    controls = fig.workbench_controls
    controls['advanced']()
    editor = controls['popups'][-1].workbench_controls
    editor['entries']['safety_margin_m'].set_val('0.4')
    editor['apply']._observers.process('clicked', None)
    assert controls['current_bench']() is bench
    confirm = controls['popups'][-1].workbench_controls
    confirm['confirm']._observers.process('clicked', None)
    assert controls['current_bench']().baseline.safety_margin_m == 0.4
    assert not controls['current_bench']().active_selection
    plt.close('all')


def test_latest_map_export_preview_never_activates_selection(tmp_path, monkeypatch):
    import matplotlib.pyplot as plt
    bench = fixture_bench(tmp_path)
    monkeypatch.setattr(opt, 'optimize_global_raceline', fake_solver)
    bench.run_baseline()
    selected = bench.select_export()
    new = wb.Workbench(bench.identity['map_image'], bench.identity['map_yaml'], tmp_path / 'fresh')
    assert new.latest_export['path'] == selected['raceline_csv']
    assert not new.active_selection and not new.candidates and not new.outputs
    fig = create_workbench_figure(new)
    assert fig.workbench_controls['buttons']['Open Previous Session'].active
    assert fig.workbench_controls['path_box'].text == selected['raceline_csv']
    assert not fig.workbench_controls['buttons']['Select / Export Validated'].active
    fig.workbench_controls['buttons']['Open Previous Session']._observers.process('clicked', None)
    assert fig.workbench_controls['current_bench']().active_selection == selected
    plt.close('all')


def test_latest_changed_export_is_not_replaced_by_an_older_one(tmp_path, monkeypatch):
    bench = fixture_bench(tmp_path)
    monkeypatch.setattr(opt, 'optimize_global_raceline', fake_solver)
    bench.run_baseline()
    selected = bench.select_export()
    Path(selected['raceline_csv']).write_text('changed')
    observed = wb.latest_export(bench.identity, [tmp_path])
    assert observed['source_session'] == str(bench.output) and 'hash mismatch' in observed['error']
    changed_identity = dict(bench.identity, map_yaml_sha256='different')
    assert wb.latest_export(changed_identity, [tmp_path]) is None


def test_finish_can_select_save_and_exit_without_losing_files(tmp_path, monkeypatch):
    import matplotlib.pyplot as plt
    bench = fixture_bench(tmp_path)
    monkeypatch.setattr(opt, 'optimize_global_raceline', fake_solver)
    bench.run_baseline()
    fig = create_workbench_figure(bench)
    fig.workbench_controls['finish']()
    dialog = fig.workbench_controls['popups'][-1]
    assert dialog.workbench_controls['save'].active
    dialog.workbench_controls['save']._observers.process('clicked', None)
    assert not plt.fignum_exists(fig.number)
    assert resolve_selection(bench.output / 'selected_raceline.yaml')['status'] == 'validated'
    assert (bench.output / 'raceline_points_optimized.csv').exists()


def test_finish_failed_candidate_does_not_save_or_delete(tmp_path, monkeypatch):
    bench = fixture_bench(tmp_path)
    failed_candidate(bench, monkeypatch)
    fig = create_workbench_figure(bench)
    fig.workbench_controls['finish']()
    dialog = fig.workbench_controls['popups'][-1]
    assert not dialog.workbench_controls['save'].active
    dialog.workbench_controls['exit']._observers.process('clicked', None)
    assert Path(bench.selected['path']).exists()
    assert not (bench.output / 'selected_raceline.yaml').exists()
