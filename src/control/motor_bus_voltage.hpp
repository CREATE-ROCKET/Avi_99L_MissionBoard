#pragma once

namespace control::motor_bus_voltage {

enum class Status {
  uninitialized,
  live,
  invalid,
};

struct Snapshot {
  Status status{Status::uninitialized};
  double voltage_v{};
};

// 校正済みADCのmotor bus電圧をproduction TorqueMapperへ公開する。
// battery present/undervoltage閾値はここでは判定しない。
void publish(double voltage_v);

// ADC read/calibrationが失敗した場合に、過去値を無期限に再利用しない。
void invalidate();

// driver end/test reset用。uninitialized時だけcaller supplied nominal Vbusを許す。
void reset();

[[nodiscard]] Snapshot snapshot();

} // namespace control::motor_bus_voltage
