cmake_minimum_required(VERSION 3.16)
project(avi_99l_mission_host_tests LANGUAGES CXX)

enable_testing()

add_executable(mission_host_tests
  mission_host_tests.cpp
  ../src/actuators/safety_core.cpp
  ../src/control/control_pipeline.cpp
  ../src/mission/command_executor.cpp
  ../src/mission/mission_state.cpp
  ../src/mission/recovery.cpp
  ../src/protocol/can_protocol.cpp
  ../src/protocol/quantization.cpp
  ../src/sensors/air_data_flight_logic.cpp
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
