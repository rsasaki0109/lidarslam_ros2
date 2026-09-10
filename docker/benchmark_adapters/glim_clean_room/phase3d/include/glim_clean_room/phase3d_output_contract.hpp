#pragma once

#include "glim_clean_room/contract.hpp"

#include <cstdint>

namespace glim_clean_room::phase3d {

// PointCloud2 row_step is a uint32 wire field.  This check is kept outside
// the ROS node so overflow behavior is testable without a running graph.
Result<std::uint32_t> checked_row_step(std::uint32_t width,
                                       std::uint32_t point_step);

}  // namespace glim_clean_room::phase3d
