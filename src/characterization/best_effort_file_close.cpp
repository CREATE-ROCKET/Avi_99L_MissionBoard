#include "characterization/best_effort_file_close.hpp"

namespace avi::characterization {
namespace {

void remember(esp_err_t operation, esp_err_t &first_error) noexcept {
  if (first_error == ESP_OK && operation != ESP_OK)
    first_error = operation;
}

} // 無名名前空間

esp_err_t bestEffortFinalizeFile(const FileCloseOperations &operations,
                                 const std::uint8_t *footer,
                                 std::size_t footer_size,
                                 esp_err_t first_error) noexcept {
  if (operations.flush == nullptr || operations.sync == nullptr ||
      operations.close == nullptr ||
      (footer_size != 0U &&
       (footer == nullptr || operations.write == nullptr)))
    return ESP_ERR_INVALID_ARG;
  if (footer_size != 0U)
    remember(operations.write(operations.context, footer, footer_size),
             first_error);
  remember(operations.flush(operations.context), first_error);
  remember(operations.sync(operations.context), first_error);
  remember(operations.close(operations.context), first_error);
  return first_error;
}

} // 名前空間 avi::characterization
