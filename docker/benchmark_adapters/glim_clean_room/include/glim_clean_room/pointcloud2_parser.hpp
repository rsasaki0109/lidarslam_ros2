#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "glim_clean_room/contract.hpp"

namespace glim_clean_room {

struct PointCloud2FieldView {
  std::string name;
  std::uint32_t offset{0};
  PointFieldType datatype{PointFieldType::kFloat32};
  std::uint32_t count{1};
};

struct PointCloud2View {
  EventStamp event{};
  std::string frame_id;
  bool is_bigendian{false};
  std::uint32_t height{0};
  std::uint32_t width{0};
  std::uint32_t point_step{0};
  std::uint32_t row_step{0};
  std::vector<PointCloud2FieldView> fields;
  std::vector<std::uint8_t> data;
};

Result<LidarFrame> parse_pointcloud2(const PointCloud2View& view,
                                     const PointFieldMapping& mapping);

}  // namespace glim_clean_room
