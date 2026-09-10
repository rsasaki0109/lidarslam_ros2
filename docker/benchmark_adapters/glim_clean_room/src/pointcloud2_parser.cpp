#include "glim_clean_room/pointcloud2_parser.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace glim_clean_room {
namespace {

std::size_t field_width(PointFieldType type) {
  switch (type) {
    case PointFieldType::kUInt16:
      return 2;
    case PointFieldType::kUInt32:
    case PointFieldType::kFloat32:
      return 4;
    case PointFieldType::kFloat64:
      return 8;
  }
  return 0;
}

bool range_fits(std::uint64_t offset, std::uint64_t width,
                std::uint64_t limit) {
  return offset <= limit && width <= limit - offset;
}

std::uint64_t read_unsigned(const std::uint8_t* bytes, std::size_t width,
                            bool bigendian) {
  std::uint64_t value = 0;
  if (bigendian) {
    for (std::size_t i = 0; i < width; ++i) {
      value = (value << 8U) | bytes[i];
    }
  } else {
    for (std::size_t i = 0; i < width; ++i) {
      value |= static_cast<std::uint64_t>(bytes[i]) << (8U * i);
    }
  }
  return value;
}

double read_number(const std::uint8_t* bytes, PointFieldType type,
                   bool bigendian) {
  const auto width = field_width(type);
  if (type == PointFieldType::kUInt16 || type == PointFieldType::kUInt32) {
    return static_cast<double>(read_unsigned(bytes, width, bigendian));
  }
  std::uint64_t raw = read_unsigned(bytes, width, bigendian);
  if (type == PointFieldType::kFloat32) {
    std::uint32_t raw32 = static_cast<std::uint32_t>(raw);
    float value = 0.0F;
    std::memcpy(&value, &raw32, sizeof(value));
    return static_cast<double>(value);
  }
  double value = 0.0;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

Result<PointCloud2FieldView> required_field(
    const std::unordered_map<std::string, PointCloud2FieldView>& fields,
    const PointFieldSpec& spec) {
  const std::string& name = spec.name;
  const auto found = fields.find(name);
  if (found == fields.end()) {
    return Result<PointCloud2FieldView>::failure(
        ErrorCode::kMissingField, "PointCloud2 is missing required field: " + name);
  }
  if (found->second.datatype != spec.datatype ||
      found->second.count != spec.count) {
    return Result<PointCloud2FieldView>::failure(
        ErrorCode::kUnsupportedFieldType,
        "PointCloud2 field does not match the declared sequence mapping: " +
            name);
  }
  return Result<PointCloud2FieldView>::success(found->second);
}

Status validate_numeric(const PointRecord& point, std::size_t index) {
  const double values[] = {point.x, point.y, point.z, point.intensity,
                           point.relative_time, point.relative_time_raw};
  for (double value : values) {
    if (!std::isfinite(value)) {
      return Status::failure(ErrorCode::kNonFinite,
                             "non-finite PointCloud2 value at point " +
                                 std::to_string(index));
    }
  }
  if (point.relative_time < 0.0) {
    return Status::failure(ErrorCode::kNonFinite,
                           "negative relative point time at point " +
                               std::to_string(index));
  }
  return Status::success();
}

}  // namespace

Result<LidarFrame> parse_pointcloud2(const PointCloud2View& view,
                                     const PointFieldMapping& mapping) {
  const auto mapping_status = validate_point_field_mapping(mapping);
  if (!mapping_status) {
    return Result<LidarFrame>::failure(mapping_status.error.code,
                                       mapping_status.error.detail);
  }
  if (view.frame_id.empty()) {
    return Result<LidarFrame>::failure(ErrorCode::kInvalidLayout,
                                       "PointCloud2 frame_id is empty");
  }
  if (view.width == 0 || view.height == 0) {
    return Result<LidarFrame>::failure(ErrorCode::kEmptyPayload,
                                       "PointCloud2 has no points");
  }
  if (view.point_step == 0 ||
      static_cast<std::uint64_t>(view.row_step) <
          static_cast<std::uint64_t>(view.point_step) * view.width) {
    return Result<LidarFrame>::failure(ErrorCode::kInvalidLayout,
                                       "PointCloud2 row/point step is invalid");
  }
  const std::uint64_t expected_bytes =
      static_cast<std::uint64_t>(view.row_step) * view.height;
  if (expected_bytes != view.data.size()) {
    return Result<LidarFrame>::failure(
        ErrorCode::kInvalidLayout,
        "PointCloud2 data length does not equal row_step*height");
  }

  std::unordered_map<std::string, PointCloud2FieldView> fields;
  struct FieldRange {
    std::uint64_t begin;
    std::uint64_t end;
    std::string name;
  };
  std::vector<FieldRange> ranges;
  for (const auto& field : view.fields) {
    if (field.name.empty()) {
      return Result<LidarFrame>::failure(
          ErrorCode::kInvalidLayout, "PointCloud2 field name is empty");
    }
    if (fields.find(field.name) != fields.end()) {
      return Result<LidarFrame>::failure(
          ErrorCode::kDuplicateField,
          "PointCloud2 contains duplicate field: " + field.name);
    }
    const std::size_t width = field_width(field.datatype);
    if (field.count == 0 || width == 0 ||
        field.count > std::numeric_limits<std::uint64_t>::max() / width ||
        !range_fits(field.offset,
                    static_cast<std::uint64_t>(width) * field.count,
                    view.point_step)) {
      return Result<LidarFrame>::failure(
          ErrorCode::kInvalidLayout,
          "PointCloud2 field has an invalid name/count/type/offset");
    }
    fields.emplace(field.name, field);
    const auto begin = static_cast<std::uint64_t>(field.offset);
    const auto end = begin + static_cast<std::uint64_t>(width) * field.count;
    for (const auto& prior : ranges) {
      if (begin < prior.end && prior.begin < end) {
        return Result<LidarFrame>::failure(
            ErrorCode::kInvalidLayout,
            "PointCloud2 fields overlap: " + prior.name + " and " +
                field.name);
      }
    }
    ranges.push_back(FieldRange{begin, end, field.name});
  }

  const auto x = required_field(fields, mapping.x);
  const auto y = required_field(fields, mapping.y);
  const auto z = required_field(fields, mapping.z);
  const auto intensity = required_field(fields, mapping.intensity);
  const auto ring = required_field(fields, mapping.ring);
  const auto time = required_field(fields, mapping.relative_time);
  for (const auto* field : {&x, &y, &z, &intensity, &ring, &time}) {
    if (!*field) {
      return Result<LidarFrame>::failure(field->error.code, field->error.detail);
    }
  }
  const std::uint64_t point_count =
      static_cast<std::uint64_t>(view.width) * view.height;
  if (point_count > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
    return Result<LidarFrame>::failure(ErrorCode::kInvalidLayout,
                                       "PointCloud2 point count overflows host size");
  }
  LidarFrame frame;
  frame.event = view.event;
  frame.frame_id = view.frame_id;
  frame.point_time_unit = mapping.time_unit;
  frame.field_mapping = mapping;
  frame.points.reserve(static_cast<std::size_t>(point_count));
  for (std::uint32_t row = 0; row < view.height; ++row) {
    for (std::uint32_t column = 0; column < view.width; ++column) {
      const std::uint64_t base = static_cast<std::uint64_t>(row) * view.row_step +
                                 static_cast<std::uint64_t>(column) * view.point_step;
      const auto at = [&](const PointCloud2FieldView& field) {
        return view.data.data() + base + field.offset;
      };
      PointRecord point;
      point.x = read_number(at(x.value), x.value.datatype, view.is_bigendian);
      point.y = read_number(at(y.value), y.value.datatype, view.is_bigendian);
      point.z = read_number(at(z.value), z.value.datatype, view.is_bigendian);
      point.intensity = read_number(at(intensity.value), intensity.value.datatype,
                                    view.is_bigendian);
      point.relative_time_raw =
          read_number(at(time.value), time.value.datatype, view.is_bigendian);
      point.relative_time =
          point_time_to_seconds(point.relative_time_raw, mapping.time_unit);
      point.ring = static_cast<std::uint32_t>(
          read_number(at(ring.value), ring.value.datatype, view.is_bigendian));
      const auto status = validate_numeric(point, frame.points.size());
      if (!status) {
        return Result<LidarFrame>::failure(status.error.code, status.error.detail);
      }
      frame.points.push_back(point);
    }
  }
  return Result<LidarFrame>::success(std::move(frame));
}

}  // namespace glim_clean_room
