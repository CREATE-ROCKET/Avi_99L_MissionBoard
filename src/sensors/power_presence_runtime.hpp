#pragma once

#include <cstdint>

namespace sensors::power_presence_runtime {

void observeRaw(uint8_t logic_voltage_raw, uint8_t motor_voltage_raw,
                uint64_t timestamp_us);
[[nodiscard]] bool logicPresent(uint64_t now_us);
[[nodiscard]] bool motorPresent(uint64_t now_us);

} // namespace sensors::power_presence_runtime
