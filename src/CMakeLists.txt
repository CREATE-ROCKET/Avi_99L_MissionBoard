idf_component_register(
  SRCS
    "main.cpp"
    "actuators/production_motor.cpp"
    "actuators/parachute_configuration.cpp"
    "actuators/safety_core.cpp"
    "bringup/safe_outputs.cpp"
    "bringup/spi_bringup.cpp"
    "bringup/encoder_bringup.cpp"
    "bringup/imu_bringup.cpp"
    "bringup/power_bringup.cpp"
    "bringup/stream_protocol.cpp"
    "control/control_pipeline.cpp"
    "control/fin_overtravel_guard.cpp"
    "control/motor_bus_voltage.cpp"
    "mission/command_executor.cpp"
    "mission/mission_state.cpp"
    "mission/recovery.cpp"
    "protocol/can_protocol.cpp"
    "protocol/quantization.cpp"
    "runtime/flight_log.cpp"
    "runtime/flight_storage.cpp"
    "runtime/imu_heartbeat_clock.cpp"
    "runtime/production_runtime.cpp"
    "runtime/recovery_boot.cpp"
    "sensors/air_data_flight_logic.cpp"
    "sensors/airspeed_estimator.cpp"
    "sensors/attitude_estimator.cpp"
    "sensors/flight_detectors.cpp"
    "sensors/power_presence.cpp"
    "sensors/power_presence_runtime.cpp"
  INCLUDE_DIRS "." "../include"
)

if(AVI_99L_CHARACTERIZATION_BUILD)
  target_compile_options(${COMPONENT_LIB} PRIVATE -O3)
endif()
