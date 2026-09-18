cmake_minimum_required(VERSION 3.5)
project(oudtra_driver_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/run_localization_smoke.py scripts/analyze_localization_run.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/localization_recording.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(localization_measurements_test
    test/test_localization_measurements.py
    TIMEOUT 30
  )
  ament_add_pytest_test(localization_smoke_test
    test/test_localization_smoke.py
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
endif()

ament_package()
