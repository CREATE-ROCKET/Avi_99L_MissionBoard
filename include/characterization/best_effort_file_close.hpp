#pragma once

#include "characterization/platform_compat.hpp"

#include <cstddef>
#include <cstdint>

namespace avi::characterization {

struct FileCloseOperations {
  void *context{nullptr};
  esp_err_t (*write)(void *context, const std::uint8_t *data,
                     std::size_t size){nullptr};
  esp_err_t (*flush)(void *context){nullptr};
  esp_err_t (*sync)(void *context){nullptr};
  esp_err_t (*close)(void *context){nullptr};
};

// 先行errorがあってもfooter、flush、sync、closeをbest-effortで全て試す。
[[nodiscard]] esp_err_t bestEffortFinalizeFile(
    const FileCloseOperations &operations, const std::uint8_t *footer,
    std::size_t footer_size, esp_err_t first_error) noexcept;

} // 名前空間 avi::characterization
