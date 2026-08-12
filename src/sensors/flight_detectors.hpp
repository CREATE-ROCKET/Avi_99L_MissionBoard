#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace sensors {

class ImuLiftoffDetector {
public:
  void reset();
  [[nodiscard]] bool update(double ax_g, double ay_g, double az_g, bool valid);

private:
  std::array<double, 20> ax_{};
  std::array<double, 20> ay_{};
  std::array<double, 20> az_{};
  std::size_t head_{};
  std::size_t count_{};
  uint8_t consecutive_{};
};

class LpsLiftoffDetector {
public:
  void reset();
  [[nodiscard]] bool update(double pressure_hpa, bool valid);

private:
  std::array<double, 5> samples_{};
  std::size_t head_{};
  std::size_t count_{};
  double previous_mean_{};
  bool have_previous_mean_{};
  uint8_t consecutive_{};
};

class PressureApexDetector {
public:
  void reset();
  [[nodiscard]] bool update(double pressure_hpa, bool valid,
                            uint64_t elapsed_since_liftoff_us);

private:
  std::array<double, 5> samples_{};
  std::size_t head_{};
  std::size_t count_{};
  double previous_mean_{};
  bool have_previous_mean_{};
  uint8_t consecutive_{};
};

} // 名前空間 sensors
