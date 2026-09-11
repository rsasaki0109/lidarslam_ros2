#include "glim_clean_room/phase3d_output_contract.hpp"

#include <limits>

namespace glim_clean_room::phase3d {

Result<std::uint32_t> checked_row_step(std::uint32_t width,
                                       std::uint32_t point_step) {
  if (width == 0 || point_step == 0) {
    return Result<std::uint32_t>::failure(
        ErrorCode::kInvalidLayout,
        "PointCloud2 width and point_step must be nonzero");
  }
  if (width > std::numeric_limits<std::uint32_t>::max() / point_step) {
    return Result<std::uint32_t>::failure(
        ErrorCode::kLedgerOverflow,
        "PointCloud2 row_step overflows its uint32 wire field");
  }
  return Result<std::uint32_t>::success(width * point_step);
}

}  // namespace glim_clean_room::phase3d
