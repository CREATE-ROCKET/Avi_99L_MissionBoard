#pragma once

#include "AS5047D.h"
#include "esp_err.h"

namespace sensors::as5047d_health {

constexpr bool statusResponseAllZero(const AS5047D::Status &status) {
  return !status.magnetic_too_low && !status.magnetic_too_high &&
         !status.cordic_overflow && !status.offset_compensation_finished &&
         status.agc == 0 && status.magnitude == 0;
}

constexpr bool statusFaulted(const AS5047D::Status &status) {
  return status.magnetic_too_low || status.magnetic_too_high ||
         status.cordic_overflow || statusResponseAllZero(status);
}

inline esp_err_t validateStatus(esp_err_t transport_result,
                                const AS5047D::Status &status) {
  // MISO stuck-lowではparityを含む全responseが0となりdriver上は成功し得る。
  // 角度0自体は正常値なので、DIAG/AGC/magnitude全体が0のstatusだけを拒否する。
  return transport_result == ESP_OK && statusResponseAllZero(status)
             ? ESP_ERR_INVALID_RESPONSE
             : transport_result;
}

static_assert(statusResponseAllZero(AS5047D::Status{}));
static_assert(!statusResponseAllZero(
    AS5047D::Status{false, false, false, true, 0, 0}));
static_assert(!statusResponseAllZero(
    AS5047D::Status{false, false, false, false, 0, 1}));
static_assert(statusFaulted(AS5047D::Status{true, false, false, false, 0, 0}));
static_assert(statusFaulted(AS5047D::Status{false, true, false, false, 0, 0}));
static_assert(statusFaulted(AS5047D::Status{false, false, true, false, 0, 0}));

} // 名前空間 sensors::as5047d_health
