cmake_minimum_required(VERSION 3.16)
project(avi_99l_mission_host_tests LANGUAGES CXX)

enable_testing()

add_executable(mission_host_tests
  mission_host_tests.cpp
  ../src/actuators/safety_core.cpp
  ../src/control/control_pipeline.cpp
  ../src/control/fin_overtravel_guard.cpp
  ../src/mission/command_executor.cpp
  ../src/mission/mission_state.cpp
  ../src/mission/recovery.cpp
  ../src/protocol/can_protocol.cpp
  ../src/protocol/quantization.cpp
  ../src/sensors/air_data_flight_logic.cpp
  ../src/sensors/airspeed_estimator.cpp
  ../src/sensors/attitude_estimator.cpp
  ../src/sensors/flight_detectors.cpp
  ../src/sensors/sensor_health.cpp
)
target_include_directories(mission_host_tests PRIVATE ../src)
target_compile_features(mission_host_tests PRIVATE cxx_std_17)
target_compile_options(mission_host_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_compile_definitions(mission_host_tests PRIVATE
  GOLDEN_VECTOR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../testdata/99l_protocol_golden_vectors.txt"
)

add_test(NAME mission_host_tests COMMAND mission_host_tests)

add_executable(fin_overtravel_tests
  fin_overtravel_tests.cpp
  ../src/control/control_pipeline.cpp
  ../src/control/fin_overtravel_guard.cpp
  ../src/mission/command_executor.cpp
  ../src/mission/mission_state.cpp
)
target_include_directories(fin_overtravel_tests PRIVATE ../src)
target_compile_features(fin_overtravel_tests PRIVATE cxx_std_17)
target_compile_options(fin_overtravel_tests PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)

add_test(NAME fin_overtravel_tests COMMAND fin_overtravel_tests)

add_executable(parachute_configuration_tests
  parachute_configuration_tests.cpp
)
target_include_directories(parachute_configuration_tests PRIVATE ../src)
target_compile_features(parachute_configuration_tests PRIVATE cxx_std_17)
target_compile_options(parachute_configuration_tests PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)

add_test(NAME parachute_configuration_tests COMMAND parachute_configuration_tests)

add_executable(mission_state_safety_regression_tests
  mission_state_safety_regression_tests.cpp
  ../src/control/fin_overtravel_guard.cpp
  ../src/mission/mission_state.cpp
)
target_include_directories(mission_state_safety_regression_tests PRIVATE ../src)
target_compile_features(mission_state_safety_regression_tests PRIVATE cxx_std_17)
target_compile_options(mission_state_safety_regression_tests PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)

add_test(NAME mission_state_safety_regression_tests
         COMMAND mission_state_safety_regression_tests)

add_executable(flight_log_tests
  flight_log_tests.cpp
  ../src/runtime/flight_log.cpp
)
target_include_directories(flight_log_tests PRIVATE ../src)
target_compile_features(flight_log_tests PRIVATE cxx_std_17)
target_compile_options(flight_log_tests PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)

add_test(NAME flight_log_tests COMMAND flight_log_tests)
