cmake_minimum_required(VERSION 3.5)
project(oudtra_driver_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/prepare_localization_overlay.py
  scripts/run_localization_smoke.py
  scripts/reanalyze_localization_run.py
  scripts/analyze_localization_run.py
  scripts/explain_pf_health.py
  scripts/analyze_sim_timing.py
  scripts/compare_localization_runs.py
  scripts/compare_matched_sections.py
  scripts/import_onboard_bag.py
  scripts/record_onboard_localization.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/localization_recording.py scripts/recording_support.py scripts/evaluation_evidence.py scripts/evaluation_control_plots.py scripts/evaluation_particle_spread.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Report-only uncertainty adapter and terminology/backup contracts.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(particle_spread_reports_test
    test/test_particle_spread_reports.py
    TIMEOUT 60
  )
  ament_add_pytest_test(control_mode_plots_test test/test_control_mode_plots.py TIMEOUT 60)
  ament_add_pytest_test(pf_health_explanation_test test/test_pf_health_explanation.py TIMEOUT 30)
  ament_add_pytest_test(matched_sections_test test/test_matched_sections.py TIMEOUT 60)
  ament_add_pytest_test(shared_evidence_test test/test_shared_evidence.py TIMEOUT 60)
  ament_add_pytest_test(onboard_import_test test/test_onboard_import.py TIMEOUT 30)
  ament_add_pytest_test(manual_localization_sim_test
    test/test_manual_localization_sim.py
    TIMEOUT 30
  )
  ament_add_pytest_test(evaluation_comparison_test
    test/test_evaluation_comparison.py
    TIMEOUT 30
  )
  ament_add_pytest_test(localization_measurements_test
    test/test_localization_measurements.py
    TIMEOUT 30
  )
  ament_add_pytest_test(localization_smoke_test
    test/test_localization_smoke.py
    TIMEOUT 30
  )
  ament_add_pytest_test(sim_timing_test
    test/test_sim_timing.py
    TIMEOUT 30
  )
  ament_add_pytest_test(full_stack_sim_defaults_test
    test/test_full_stack_sim_defaults.py
    TIMEOUT 30
  )
  ament_add_pytest_test(onboard_profile_contracts_test
    test/test_onboard_profile_contracts.py
    TIMEOUT 30
  )
  ament_add_pytest_test(onboard_recording_test
    test/test_onboard_recording.py
    TIMEOUT 30
  )
endif()

ament_package()
